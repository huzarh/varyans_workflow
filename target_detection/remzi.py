#!/usr/bin/env python3
"""
Optimize edilmiş target detection sistemi
"""
import sys
import platform
import os
import math
import json
import time
from typing import List, Dict, Any, Optional, Tuple

# Görüntü işleme
import cv2
import numpy as np

# MAVLink
from pymavlink import mavutil

# # GUI - COMMENTED FOR HEADLESS OPERATION
# from PySide6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
# from PySide6.QtGui import QImage, QPixmap, QFont
# from PySide6.QtCore import Qt, QTimer, QThread, Signal

# Kendi modüllerimiz
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.state_manager import StateManager
from core.config import camera_config, detection_config
from flight.controller import UavController
from mode.mavlink_func import switch_to_guided
from core.stream_server import StreamServer

# ----------------- Kamera Soyutlama -----------------
PICAM_AVAILABLE = False
try:
    from picamera2 import Picamera2
    PICAM_AVAILABLE = True
except ImportError:
    PICAM_AVAILABLE = False

class FrameGrabber:
    """Optimize edilmiş kamera sınıfı"""
    
    def __init__(self, width: int = None, height: int = None, fps: int = None):
        self.width = width or camera_config.width
        self.height = height or camera_config.height
        self.fps = fps or camera_config.fps
        self.source = None
        self.picam = None
        self.cap = None
        self._open()
    
    def _open(self):
        """Kamera kaynağını aç"""
        if PICAM_AVAILABLE:
            try:
                self.picam = Picamera2()
                cfg = self.picam.create_video_configuration(
                    main={"size": (self.width, self.height), "format": "BGR888"}
                )
                self.picam.configure(cfg)
                self.picam.start()
                self.source = "picamera2"
                return
            except Exception as e:
                print(f"Picamera2 açılamadı, V4L2'ye düşülüyor: {e}")
        
        # V4L2 fallback
        backend = cv2.CAP_V4L2 if platform.system() != "Windows" else cv2.CAP_DSHOW
        self.cap = cv2.VideoCapture(0, backend)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Kamera açılamadı (ne Picamera2 ne V4L2).")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.source = "opencv"
    
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Frame oku"""
        try:
            if self.source == "picamera2":
                arr = self.picam.capture_array()
                frame_bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return True, frame_bgr
            else:
                return self.cap.read()
        except Exception as e:
            print(f"Kare okunamadı: {e}")
            return False, None
    
    def release(self):
        """Kamera kaynağını serbest bırak"""
        if self.source == "picamera2" and self.picam is not None:
            try:
                self.picam.stop()
            except Exception:
                pass
        if self.source == "opencv" and self.cap is not None:
            self.cap.release()

# ----------------- Geometri & Mesafe -----------------
def angle_cos(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    """Açı kosinüsünü hesapla"""
    d1, d2 = p0 - p1, p2 - p1
    return float(np.dot(d1, d2) / (np.linalg.norm(d1) * np.linalg.norm(d2) + 1e-8))

def is_square(contour: np.ndarray, min_area: int = 400, angle_tol: float = 0.20, aspect_tol: float = 0.20) -> bool:
    """Konturun kare olup olmadığını kontrol et"""
    area = cv2.contourArea(contour)
    if area < min_area:
        return False
    
    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return False
    
    pts = approx.reshape(-1, 2).astype(np.float32)
    max_cos = max(abs(angle_cos(pts[(i-1)%4], pts[i], pts[(i+1)%4])) for i in range(4))
    if max_cos > angle_tol:
        return False
    
    x, y, w, h = cv2.boundingRect(approx)
    if h == 0:
        return False
    
    return abs((w / float(h)) - 1.0) <= aspect_tol

def approx_square_corners(contour: np.ndarray) -> Optional[np.ndarray]:
    """Kare köşelerini yaklaşık olarak bul"""
    eps = 0.02 * cv2.arcLength(contour, True)
    ap = cv2.approxPolyDP(contour, eps, True)
    if ap is None or len(ap) != 4:
        return None
    
    pts = ap.reshape(-1, 2).astype(np.float32)
    c = np.mean(pts, axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(ang)
    return pts[order]

def side_length_pixels(pts: np.ndarray) -> Optional[float]:
    """Kenar uzunluğunu pixel cinsinden hesapla"""
    if pts is None or len(pts) != 4:
        return None
    
    distances = [
        np.linalg.norm(pts[0] - pts[1]),
        np.linalg.norm(pts[1] - pts[2]),
        np.linalg.norm(pts[2] - pts[3]),
        np.linalg.norm(pts[3] - pts[0])
    ]
    return float(np.mean(distances))

def ensure_focal_from_fov(frame_w: int) -> None:
    """Focal length'i FOV'tan hesapla"""
    global FOCAL_PX
    if camera_config.hfov_deg and FOCAL_PX is None and frame_w > 0:
        FOCAL_PX = (0.5 * frame_w) / math.tan(math.radians(camera_config.hfov_deg * 0.5))

def distance_from_altitude(pixel: Tuple[float, float], frame_shape: Tuple[int, int, int], H_m: float) -> Tuple[Optional[float], Optional[float]]:
    """İrtifadan mesafe hesapla"""
    h, w = frame_shape[:2]
    ensure_focal_from_fov(w)
    if FOCAL_PX is None or H_m is None:
        return None, None
    
    u, v = float(pixel[0]), float(pixel[1])
    cx, cy = w / 2.0, h / 2.0
    f = float(FOCAL_PX)
    x = (u - cx) / f
    y = (v - cy) / f
    ground_range = H_m * math.sqrt(x * x + y * y)
    slant = H_m * math.sqrt(x * x + y * y + 1.0)
    return slant, ground_range

def estimate_distance_m_robust(contour: np.ndarray, frame_shape: Tuple[int, int, int]) -> Tuple[Optional[float], Optional[float], Optional[np.ndarray]]:
    """Robust mesafe tahmini"""
    h, w = frame_shape[:2]
    ensure_focal_from_fov(w)
    pts = approx_square_corners(contour)
    if pts is None:
        return None, None, None
    
    f = FOCAL_PX if FOCAL_PX else 800.0
    cx, cy = w / 2.0, h / 2.0
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float32)
    dist = np.zeros((5, 1), dtype=np.float32)
    s = camera_config.real_size_m / 2.0
    objp = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)
    imgp = pts.astype(np.float32)
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=flags)
    if ok:
        Z = float(np.linalg.norm(tvec))
    else:
        side_px = side_length_pixels(pts)
        if side_px is not None and side_px > 1:
            Z = (f * camera_config.real_size_m) / side_px
        else:
            Z = None
    
    side_px = side_length_pixels(pts)
    return Z, side_px, pts

def bbox_area_px2(contour: np.ndarray) -> int:
    """Bounding box alanını hesapla"""
    x, y, w, h = cv2.boundingRect(contour)
    return int(w * h)

def compute_confidence(target_dict: Dict[str, Any], frame_shape: Tuple[int, int, int]) -> float:
    """Güven skorunu hesapla"""
    h, w = frame_shape[:2]
    vis = min(target_dict.get("visible_frames", 0) / max(detection_config.lock_threshold, 1), 1.0)
    c = target_dict.get("contour", None)
    if c is None:
        return round(float(0.4 * vis), 2)
    
    pts = approx_square_corners(c)
    area = cv2.contourArea(c)
    
    # Basit güven hesaplama
    area_norm = max(0.0, min(area / 8000.0, 1.0))
    cx, cy = target_dict["cx"], target_dict["cy"]
    offset = math.hypot(cx - w/2, cy - h/2) / (0.5 * w)
    offset_score = float(max(0.4, 1.0 - 0.6 * offset))
    
    conf = vis * area_norm * offset_score
    return round(float(max(min(conf, 1.0), 0.0)), 2)

# Global değişkenler
FOCAL_PX = None
kernel = np.ones((5, 5), np.uint8)
alpha_mask = 1

# ----------------- Asenkron İşçi - COMMENTED FOR HEADLESS OPERATION -----------------
# class ActionWorker(QThread):
#     """Optimize edilmiş action worker"""
#     done = Signal(str)
#     
#     def __init__(self, state_manager: StateManager, parent=None):
#         super().__init__(parent)
#         self.state_manager = state_manager
#         self.controller = UavController()
#         self._queue = []
#         self._running = True
#         self._last_action_ts = 0.0
#     
#     def queue_action(self, payload: Dict[str, Any]):
#         """Action'ı kuyruğa ekle"""
#         self._queue.append(payload)
#     
#     def stop(self):
#         """Worker'ı durdur"""
#         self._running = False
#     
#     def run(self):
#         """Worker main loop"""
#         while self._running:
#             if not self._queue:
#                 self.msleep(10)
#                 continue
#             
#             payload = self._queue.pop(0)
#             now = time.time()
#             
#             if now - self._last_action_ts < detection_config.action_cooldown_sec:
#                 continue
#             
#             self._last_action_ts = now
# 
#             try:
#                 print("jhkjhkjhkj--------------GUİDED")
#                 switch_to_guided()
#             except Exception as e:
#                 self.done.emit(f"MAVLink hata: {e}")
#                 continue
#             
#             try:
#                 self.controller.guided_approach_velocity(self.state_manager)
#                 self.done.emit("Controller OK")
#             except Exception as e:
#                 self.done.emit(f"Controller hata: {e}")

# ----------------- Algılama -----------------
def detect_targets(mask: np.ndarray, min_area: int, tracked_list: List[Dict[str, Any]]) -> None:
    """Hedefleri algıla ve takip et"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current = []
    
    for c in contours:
        if cv2.contourArea(c) >= min_area and is_square(c, detection_config.min_area):
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                current.append({
                    "cx": cx, "cy": cy, "locked": False, "action_sent": False,
                    "visible_frames": 1, "lost_frames": 0, "contour": c
                })
    
    # Basit yakınlık eşleştirme
    for det in current:
        matched = False
        for t in tracked_list:
            if (abs(det["cx"] - t["cx"]) < detection_config.position_tolerance and 
                abs(det["cy"] - t["cy"]) < detection_config.position_tolerance):
                t.update({"cx": det["cx"], "cy": det["cy"]})
                t["contour"] = det["contour"]
                t["visible_frames"] += 1
                t["lost_frames"] = 0
                matched = True
                break
        if not matched:
            tracked_list.append(det)
    
    # Kayıp frame'leri işle
    for t in tracked_list:
        if all(abs(t["cx"] - d["cx"]) > detection_config.position_tolerance or 
               abs(t["cy"] - d["cy"]) > detection_config.position_tolerance for d in current):
            t["lost_frames"] += 1
    
    # Kayıp frame'leri temizle
    tracked_list[:] = [t for t in tracked_list if t["lost_frames"] <= detection_config.lost_frame_threshold]

def build_detection_info(color_name: str, target_dict: Dict[str, Any], frame_shape: Tuple[int, int, int], state: StateManager) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Algılama bilgilerini oluştur"""
    cx, cy = int(target_dict["cx"]), int(target_dict["cy"])
    contour = target_dict.get("contour", None)
    area_px2 = bbox_area_px2(contour) if contour is not None else 0
    
    # Mesafe hesaplama
    H = None  # İrtifa bilgisi yok
    dist_alt, ground_range = (None, None)
    if H is not None:
        dist_alt, ground_range = distance_from_altitude((cx, cy), frame_shape, H)
    
    dist_size, side_px, pts = (None, None, None)
    if dist_alt is None and contour is not None:
        dist_size, side_px, pts = estimate_distance_m_robust(contour, frame_shape)
    
    chosen_dist = dist_alt if dist_alt is not None else dist_size
    
    # State'i güncelle
    state.update_target(color_name, cx, cy, int(area_px2), compute_confidence(target_dict, frame_shape))
    
    info = {
        "detected": True,
        "type": f"{color_name}_4x4",
        "pixel": [cx, cy],
        "bbox_area": int(area_px2),
        "distance_m": round(float(chosen_dist), 2) if chosen_dist is not None else None,
        "target_lat": None,  # GPS bilgisi yok
        "target_lon": None,  # GPS bilgisi yok
        "confidence": compute_confidence(target_dict, frame_shape)
    }
    
    debug = {
        "side_px": side_px,
        "FOCAL_PX": FOCAL_PX,
        "HFOV_DEG": camera_config.hfov_deg,
        "altitude_m": H,
        "ground_range_m": ground_range,
        "pnp_pts": pts
    }
    
    return info, debug

# # ----------------- PySide6 Arayüz - COMMENTED FOR HEADLESS OPERATION -----------------
# class IHAInterface(QWidget):
#     """Optimize edilmiş ana arayüz"""
#     
#     def __init__(self):
#         super().__init__()
#         self._setup_ui()
#         self._setup_components()
#         self._setup_connections()
#     
#     def _setup_ui(self):
#         """UI bileşenlerini kur"""
#         self.setWindowTitle("İHA Görüntü ve TeleMETRİ Arayüzü (Optimized)")
#         self.setGeometry(40, 40, 1700, 900)
#         self.setStyleSheet("background-color: #1e1e1e; color: white;")
#         
#         # Başlık
#         title = QLabel("🚁 İHA GÖRÜNTÜ VE TELEMETRİ ARAYÜZÜ")
#         title.setFont(QFont("Arial", 20, QFont.Bold))
#         title.setAlignment(Qt.AlignCenter)
#         
#         # Video frame'leri
#         self.label_main = self._make_video_frame("Ana Görsel")
#         self.label_blue = self._make_video_frame("Mavi Maske (Kare)")
#         self.label_red = self._make_video_frame("Kırmızı Maske (Kare)")
#         
#         # Layout'lar
#         video_layout = QHBoxLayout()
#         video_layout.addWidget(self.label_main)
#         video_layout.addWidget(self.label_blue)
#         video_layout.addWidget(self.label_red)
#         
#         # Durum ve JSON
#         self.status_label = QLabel("Durum: Hazır")
#         self.status_label.setFont(QFont("Arial", 12))
#         
#         self.json_label = QLabel("{}")
#         self.json_label.setFont(QFont("Consolas", 10))
#         self.json_label.setStyleSheet("background:#111; border:1px solid #444; padding:6px;")
#         
#         # Butonlar
#         self.btn_start = QPushButton("▶ Başlat")
#         self.btn_stop = QPushButton("⏹ Durdur")
#         self.btn_exit = QPushButton("❌ Çıkış")
#         
#         for btn in [self.btn_start, self.btn_stop, self.btn_exit]:
#             btn.setFixedHeight(40)
#             btn.setStyleSheet("""
#                 QPushButton { background-color: #2e8b57; color: white; font-size: 14px; border-radius: 8px; }
#                 QPushButton:hover { background-color: #3cb371; }
#             """)
#         
#         # Ana layout
#         main_layout = QVBoxLayout()
#         main_layout.addWidget(title)
#         main_layout.addLayout(video_layout)
#         
#         bottom_layout = QHBoxLayout()
#         left_box = QVBoxLayout()
#         left_box.addWidget(self.status_label)
#         left_box.addWidget(self.json_label)
#         bottom_layout.addLayout(left_box)
#         bottom_layout.addStretch()
#         
#         button_layout = QHBoxLayout()
#         button_layout.addStretch()
#         button_layout.addWidget(self.btn_start)
#         button_layout.addWidget(self.btn_stop)
#         button_layout.addWidget(self.btn_exit)
#         
#         main_layout.addLayout(bottom_layout)
#         main_layout.addLayout(button_layout)
#         self.setLayout(main_layout)
#     
#     def _setup_components(self):
#         """Bileşenleri kur"""
#         # State manager
#         self.state_manager = StateManager()
#         
#         # Timer
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_frames)
#         
#         # Frame grabber
#         self.grabber = FrameGrabber()
#         
#         # Action worker
#         self.worker = ActionWorker(self.state_manager)
#         self.worker.done.connect(self.on_worker_done)
#         self.worker.start()
#         
#         # Takip listeleri
#         self.tracked_blue = []
#         self.tracked_red = []
#         self.prev_mask_blue = None
#         self.prev_mask_red = None
#         
#         # Debug
#         self.last_detection_json = {}
#         self.debug = {}
#         self.print_counter = 0
#     
#     def _setup_connections(self):
#         """Bağlantıları kur"""
#         self.btn_start.clicked.connect(self.start_stream)
#         self.btn_stop.clicked.connect(self.stop_stream)
#         self.btn_exit.clicked.connect(self.close)
#     
#     def _make_video_frame(self, title: str) -> QLabel:
#         """Video frame oluştur"""
#         frame = QLabel(title)
#         frame.setFixedSize(500, 400)
#         frame.setStyleSheet("background-color: black; border: 2px solid gray; color: white;")
#         frame.setAlignment(Qt.AlignCenter)
#         return frame
#     
#     def start_stream(self):
#         """Stream'i başlat"""
#         self.status_label.setText("Durum: Yayın Başladı")
#         self.timer.start(33)  # ~30 fps
#     
#     def stop_stream(self):
#         """Stream'i durdur"""
#         self.status_label.setText("Durum: Durduruldu")
#         self.timer.stop()
#     
#     def on_worker_done(self, msg: str):
#         """Worker işlemi tamamlandı"""
#         self.status_label.setText(f"Durum: {msg}")
#     
#     def set_frame(self, label: QLabel, frame: np.ndarray):
#         """Frame'i label'a set et"""
#         rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         h, w, ch = rgb.shape
#         qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
#         pix = QPixmap.fromImage(qimg).scaled(label.width(), label.height(), Qt.KeepAspectRatio)
#         label.setPixmap(pix)
#     
#     def update_frames(self):
#         """Frame'leri güncelle"""
#         ret, frame_bgr = self.grabber.read()
#         if not ret or frame_bgr is None:
#             self.status_label.setText("Durum: Kare okunamadı")
#             return
#         
#         ensure_focal_from_fov(frame_bgr.shape[1])
#         frame_bgr = cv2.flip(frame_bgr, 1)  # Aynalama
#         
#         # Karanlık kontrolü
#         hsv_for_check = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#         if hsv_for_check[..., 2].mean() < 10:
#             disp_main = frame_bgr.copy()
#             cv2.putText(disp_main, "NO IMAGE (low brightness) - skipping detection",
#                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
#             self.set_frame(self.label_main, disp_main)
#             self.set_frame(self.label_blue, np.zeros_like(frame_bgr))
#             self.set_frame(self.label_red, np.zeros_like(frame_bgr))
#             return
#         
#         # Maske oluşturma
#         hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#         mask_blue = cv2.inRange(hsv, (90, 80, 50), (130, 255, 255))
#         mask_red = cv2.inRange(hsv, (0, 180, 150), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
#         
#         # Morfoloji
#         mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, 1)
#         mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, 2)
#         mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, 1)
#         mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, 2)
#         
#         # Zamanla yumuşatma
#         if self.prev_mask_blue is not None:
#             mask_blue = cv2.addWeighted(mask_blue, alpha_mask, self.prev_mask_blue, 1 - alpha_mask, 0).astype(np.uint8)
#         self.prev_mask_blue = mask_blue.copy()
#         
#         if self.prev_mask_red is not None:
#             mask_red = cv2.addWeighted(mask_red, alpha_mask, self.prev_mask_red, 1 - alpha_mask, 0).astype(np.uint8)
#         self.prev_mask_red = mask_red.copy()
#         
#         # Hedef algılama
#         detect_targets(mask_blue, detection_config.min_area, self.tracked_blue)
#         detect_targets(mask_red, detection_config.min_area, self.tracked_red)
#         
#         # Görselleştirme
#         disp_main = frame_bgr.copy()
#         disp_blue = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
#         disp_red = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
#         
#         # Merkez çizgisi
#         ref_x, ref_y = frame_bgr.shape[1] // 2, frame_bgr.shape[0] // 2
#         cv2.circle(disp_main, (ref_x, ref_y), 6, (0, 255, 255), -1)
#         cv2.circle(disp_blue, (ref_x, ref_y), 6, (0, 255, 255), -1)
#         cv2.circle(disp_red, (ref_x, ref_y), 6, (0, 255, 255), -1)
#         
#         detection_json_to_show = None
#         self.debug = {}
#         
#         # Hedef işleme
#         for color_name, tracked_list, disp, color in [
#             ("blue", self.tracked_blue, disp_blue, (255, 0, 0)),
#             ("red", self.tracked_red, disp_red, (0, 0, 255))
#         ]:
#             for t in tracked_list:
#                 if not t["locked"] and t["visible_frames"] >= detection_config.lock_threshold:
#                     t["locked"] = True
#                 
#                 if t["locked"]:
#                     # Çizimler
#                     if t.get("contour") is not None:
#                         cv2.drawContours(disp, [t["contour"]], -1, (0, 0, 255), 2)
#                     cv2.circle(disp, (t["cx"], t["cy"]), 6, (0, 0, 255), -1)
#                     
#                     # Bilgi oluştur
#                     info, dbg = build_detection_info(color_name, t, frame_bgr.shape, self.state_manager)
#                     detection_json_to_show = info
#                     self.debug = dbg
#                     
#                     # Action gönder
#                     if not t.get("action_sent", False):
#                         self.worker.queue_action({
#                             "color": color_name,
#                             "pixel": (t["cx"], t["cy"]),
#                             "info": info
#                         })
#                         t["action_sent"] = True
#                     
#                     # Konsol çıktısı
#                     self.print_counter += 1
#                     if self.print_counter % detection_config.print_every_n == 0:
#                         try:
#                             print(json.dumps(info, ensure_ascii=False))
#                         except Exception:
#                             pass
#                 else:
#                     # Kilit yoksa küçük nokta
#                     cv2.circle(disp, (t["cx"], t["cy"]), 4, color, -1)
#         
#         # UI güncelle
#         self.set_frame(self.label_main, disp_main)
#         self.set_frame(self.label_blue, disp_blue)
#         self.set_frame(self.label_red, disp_red)
#         
#         if detection_json_to_show is not None:
#             self.last_detection_json = detection_json_to_show
#             self.json_label.setText(json.dumps(detection_json_to_show, ensure_ascii=False))
#     
#     def closeEvent(self, e):
#         """Pencere kapatılırken"""
#         try:
#             self.timer.stop()
#             self.worker.stop()
#             self.worker.wait(500)
#             self.grabber.release()
#         except Exception:
#             pass
#         return super().closeEvent(e)

# # ----------------- Çalıştır - COMMENTED FOR HEADLESS OPERATION -----------------
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     win = IHAInterface()
#     win.show()
#     sys.exit(app.exec())

# ----------------- Headless Çalıştırma -----------------
def main():
    """Raspberry Pi 5 için headless çalıştırma"""
    print("İHA Target Detection başlatılıyor...")
    
    # State manager
    state_manager = StateManager()
    
    # Frame grabber
    grabber = FrameGrabber()
    
    # Stream server başlat
    stream_server = StreamServer(host='0.0.0.0', port=8080)
    stream_server.start()
    
    # Takip listeleri
    tracked_blue = []
    tracked_red = []
    prev_mask_blue = None
    prev_mask_red = None
    
    # Debug
    print_counter = 0
    
    try:
        while True:
            ret, frame_bgr = grabber.read()
            if not ret or frame_bgr is None:
                print("Kare okunamadı")
                continue
            
            ensure_focal_from_fov(frame_bgr.shape[1])
            frame_bgr = cv2.flip(frame_bgr, 1)  # Aynalama
            
            # Karanlık kontrolü
            hsv_for_check = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            if hsv_for_check[..., 2].mean() < 10:
                print("NO IMAGE (low brightness) - skipping detection")
                # Stream server'a karanlık frame gönder
                dark_frame = frame_bgr.copy()
                cv2.putText(dark_frame, "NO IMAGE (low brightness) - skipping detection",
                           (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                stream_server.update_frame(dark_frame)
                continue
            
            # Maske oluşturma
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            mask_blue = cv2.inRange(hsv, (90, 80, 50), (130, 255, 255))
            mask_red = cv2.inRange(hsv, (0, 180, 150), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
            
            # Morfoloji
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, 1)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, 2)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, 1)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, 2)
            
            # Zamanla yumuşatma
            if prev_mask_blue is not None:
                mask_blue = cv2.addWeighted(mask_blue, alpha_mask, prev_mask_blue, 1 - alpha_mask, 0).astype(np.uint8)
            prev_mask_blue = mask_blue.copy()
            
            if prev_mask_red is not None:
                mask_red = cv2.addWeighted(mask_red, alpha_mask, prev_mask_red, 1 - alpha_mask, 0).astype(np.uint8)
            prev_mask_red = mask_red.copy()
            
            # Hedef algılama
            detect_targets(mask_blue, detection_config.min_area, tracked_blue)
            detect_targets(mask_red, detection_config.min_area, tracked_red)
            
            # Görselleştirme
            disp_main = frame_bgr.copy()
            disp_blue = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
            disp_red = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
            
            # Merkez çizgisi
            ref_x, ref_y = frame_bgr.shape[1] // 2, frame_bgr.shape[0] // 2
            cv2.circle(disp_main, (ref_x, ref_y), 6, (0, 255, 255), -1)
            cv2.circle(disp_blue, (ref_x, ref_y), 6, (0, 255, 255), -1)
            cv2.circle(disp_red, (ref_x, ref_y), 6, (0, 255, 255), -1)
            
            detection_json_to_show = None
            blue_detection_data = None
            red_detection_data = None
            
            # Hedef işleme
            for color_name, tracked_list, disp, color in [
                ("blue", tracked_blue, disp_blue, (255, 0, 0)),
                ("red", tracked_red, disp_red, (0, 0, 255))
            ]:
                for t in tracked_list:
                    if not t["locked"] and t["visible_frames"] >= detection_config.lock_threshold:
                        t["locked"] = True
                    
                    if t["locked"]:
                        # Çizimler
                        if t.get("contour") is not None:
                            cv2.drawContours(disp, [t["contour"]], -1, (0, 0, 255), 2)
                        cv2.circle(disp, (t["cx"], t["cy"]), 6, (0, 0, 255), -1)
                        
                        # Bilgi oluştur
                        info, dbg = build_detection_info(color_name, t, frame_bgr.shape, state_manager)
                        detection_json_to_show = info
                        
                        # Stream server'a detection data gönder
                        if color_name == "blue":
                            blue_detection_data = info
                        else:
                            red_detection_data = info
                        
                        # Action gönder
                        if not t.get("action_sent", False):
                            try:
                                print("jhkjhkjhkj--------------GUİDED")
                                switch_to_guided()
                                controller = UavController()
                                controller.guided_approach_velocity(state_manager)
                                print("Controller OK")
                            except Exception as e:
                                print(f"MAVLink/Controller hata: {e}")
                            t["action_sent"] = True
                        
                        # Konsol çıktısı
                        print_counter += 1
                        if print_counter % detection_config.print_every_n == 0:
                            try:
                                print(json.dumps(info, ensure_ascii=False))
                            except Exception:
                                pass
                    else:
                        # Kilit yoksa küçük nokta
                        cv2.circle(disp, (t["cx"], t["cy"]), 4, color, -1)
            
            # Stream server'a frame ve detection data gönder
            stream_server.update_frame(disp_main)
            stream_server.update_detection_data(blue_detection_data, red_detection_data)
            
            time.sleep(0.033)  # ~30 fps
            
    except KeyboardInterrupt:
        print("Program durduruldu")
    finally:
        grabber.release()
        stream_server.stop()

if __name__ == "__main__":
    main()

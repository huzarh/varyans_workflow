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
try:
    # OpenCV loglarını sessize al
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:
    pass
import numpy as np


# Kendi modüllerimiz
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.state_manager import StateManager
from core.config import camera_config, detection_config
from flight.controller import UavController
from mode.mavlink_func import switch_to_guided
from core.stream_server import StableStreamServer as StreamServer

# ----------------- Kamera Soyutlama -----------------
PICAM_AVAILABLE = False
try:
    # Ortam değişkeni ile Picamera2 devre dışı bırakılabilir
    if os.getenv("DISABLE_PICAM", "0") != "1":
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
        
        # V4L2/OpenCV fallback - birden çok index ve backend dene
        backends = []
        try:
            backends.append(cv2.CAP_V4L2)
        except Exception:
            pass
        try:
            backends.append(cv2.CAP_ANY)
        except Exception:
            pass
        if platform.system() == "Windows":
            try:
                backends.append(cv2.CAP_DSHOW)
            except Exception:
                pass
        if not backends:
            backends = [0]
        
        tried = []
        for backend in backends:
            for cam_index in [0, 1, 2, 3]:
                try:
                    cap = cv2.VideoCapture(cam_index, backend) if isinstance(backend, int) else cv2.VideoCapture(cam_index)
                    if cap is not None and cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                        self.cap = cap
                        self.source = "opencv"
                        return
                    else:
                        tried.append(f"index {cam_index} backend {backend}")
                        if cap is not None:
                            cap.release()
                except Exception:
                    tried.append(f"index {cam_index} backend {backend}")
                    try:
                        cap.release()
                    except Exception:
                        pass
        
        # Kameralar açılamadı -> sentetik kaynak kullan
        print("Uyarı: Hiçbir kamera açılamadı. Sentetik siyah frame kaynağı kullanılacak.")
        self.source = "synthetic"
    
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Frame oku"""
        try:
            if self.source == "picamera2":
                arr = self.picam.capture_array()
                frame_bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return True, frame_bgr
            if self.source == "opencv":
                return self.cap.read()
            if self.source == "synthetic":
                return True, np.zeros((self.height, self.width, 3), dtype=np.uint8)
            return False, None
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
        # synthetic için yapılacak bir şey yok

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

# ----------------- Hedef Merkezi ile Kamera Merkezi Arası Çizgi Fonksiyonu -----------------
def draw_center_line(frame: np.ndarray, target_center: Tuple[int, int], color: Tuple[int, int, int], thickness: int = 2):
    """Kamera merkezi ile hedef merkezi arasında çizgi çiz"""
    frame_height, frame_width = frame.shape[:2]
    camera_center = (frame_width // 2, frame_height // 2)
    
    # Çizgiyi çiz
    cv2.line(frame, camera_center, target_center, color, thickness)
    
    # Çizgi uclarına küçük daireler ekle (opsiyonel)
    cv2.circle(frame, camera_center, 3, color, -1)
    cv2.circle(frame, target_center, 3, color, -1)

# ----------------- Headless Çalıştırma -----------------
def main():
    """Raspberry Pi 5 için headless çalıştırma"""
    print("İHA Target Detection başlatılıyor...")
    
    # State manager
    state_manager = StateManager()
    
    # Frame grabber
    grabber = FrameGrabber()
    
    # Stream server başlat
    stream_server = None
    try:
        stream_server = StreamServer(host='0.0.0.0', port=8080)
        stream_server.start(enable_capture=False)
    except Exception as e:
        print(f"Stream server başlatılamadı: {e}")
        return
    
    # Takip listeleri
    tracked_blue = []
    tracked_red = []
    prev_mask_blue = None
    prev_mask_red = None
    
    # Debug
    print_counter = 0
    prev_time = time.time()
    current_fps = 0.0
    target_info_list = [] 

    try:
        while True:
            try:
                ret, frame_bgr = grabber.read()
                if not ret or frame_bgr is None:
                    print("Kare okunamadı")
                    continue
                
                ensure_focal_from_fov(frame_bgr.shape[1])
                frame_bgr = cv2.flip(frame_bgr, 1)  # Aynalama
                
                # Karanlık kontrolü
                hsv_for_check = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
                if hsv_for_check[..., 2].mean() < 10:
                    # Stream server'a karanlık frame gönder
                    dark_frame = frame_bgr.copy()
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
                            
                            # Hedef merkezi ile kamera merkezi arası çizgi çiz (ana frame'de)
                            target_center = (t["cx"], t["cy"])
                            
                            # Mavi hedefler için mavi çizgi, kırmızı hedefler için kırmızı çizgi
                            if color_name == "blue":
                                line_color = (255, 0, 0)  # Mavi (BGR)
                            else:  # red
                                line_color = (0, 0, 255)  # Kırmızı (BGR)
                            
                            # Ana görüntüye çizgiyi çiz
                            draw_center_line(disp_main, target_center, line_color, thickness=3)
                            
                            # Bilgi oluştur
                            info, dbg = build_detection_info(color_name, t, frame_bgr.shape, state_manager)
                            
                            # Action gönder
                            if not t.get("action_sent", False):
                                try:
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
                                    # info zaten yukarıda tanımlandı
                                    print(json.dumps(info, ensure_ascii=False))
                                except Exception:
                                    pass
                        else:
                            # Kilit yoksa küçük nokta
                            cv2.circle(disp, (t["cx"], t["cy"]), 4, color, -1)
                            cv2.circle(disp_main, (t["cx"], t["cy"]), 6, color, 1)
                            
                            # Kilitsiz hedefler için de çizgi çiz (daha ince)
                            target_center = (t["cx"], t["cy"])
                            if color_name == "blue":
                                line_color = (255, 100, 100)  # Açık mavi
                            else:  # red  
                                line_color = (100, 100, 255)  # Açık kırmızı
                            
                            draw_center_line(disp_main, target_center, line_color, thickness=1)
                            
                            # Kilitsiz hedef bilgilerini de ekle
                            target_info_list.append({
                                "color": color_name,
                                "position": [t["cx"], t["cy"]],
                                "confidence": compute_confidence(t, frame_bgr.shape),
                                "distance": None,
                                "locked": False
                            })
            except Exception as e:
                print(f"Frame işleme hatası: {e}")
                continue
            
            # FPS hesaplama
            now = time.time()
            current_fps = 1.0 / (now - prev_time) if (now - prev_time) > 0 else 0.0
            prev_time = now

            # Hedef sayıları
            total_targets = len(tracked_blue) + len(tracked_red)
            locked_targets = sum(1 for t in tracked_blue + tracked_red if t.get("locked"))

            # Ekranda FPS ve hedef bilgilerini göster
            fps_text = f"FPS: {current_fps:.1f}"
            target_text = f"Targets: {total_targets} | Locked: {locked_targets}"
            
            # FPS bilgisini ekranın sol üstüne yaz
            cv2.putText(disp_main, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 0), 2)
            
            # Hedef bilgisini FPS'nin altına yaz
            cv2.putText(disp_main, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 0), 2)
            # Her frame başında listeyi temizle
            target_info_list = []
            
            # Stream server'a frame ve meta data gönder
            stream_metadata = {
                "fps": round(current_fps, 1),
                "total_targets": total_targets,
                "locked_targets": locked_targets,
                "targets": target_info_list,
                "timestamp": time.time()
            }
            
            stream_server.update_frame(disp_main)
            # Meta data gönderimi için stream_server'ın bu özelliği desteklemesi gerekiyor
            # Eğer destekliyorsa:
            try:
                if hasattr(stream_server, 'update_metadata'):
                    stream_server.update_metadata(stream_metadata)
            except Exception as e:
                pass  # Meta data gönderilemezse sessizce devam et
            
            time.sleep(0.030)  # ~33 fps (25-35 arası)
    except KeyboardInterrupt:
        print("Program durduruldu")
    finally:
        try:
            grabber.release()
        except Exception:
            pass
        try:
            stream_server.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
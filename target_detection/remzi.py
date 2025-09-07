import sys
import platform
import os
import math
import json
import time

# göre ve amaç
import cv2
import numpy as np
from pymavlink import mavutil
from PySide6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtCore import Qt, QTimer, QThread, Signal
# kendi kodlarımız

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import mavlink_func
from flight import controller

# state manager kütüphanesi
from core.state_manager import StateManager


# ----------------- Global Ayarlar -----------------
PRINT_EVERY_N = 10                # Konsola her N kilitte bir yaz
ACTION_COOLDOWN_SEC = 1.5         # Aynı hedef için eylem tetik aralığı
REAL_SIZE_M = 0.05                # 5 cm kare
USE_ALTITUDE = True               # İsterseniz gerçek irtifa verinizle doldurun
USE_HFOV   = True
HFOV_DEG   = 66.0
FOCAL_PX   = None                 # İlk frameden sonra HFOV'tan hesaplanır

# Maske ve takip parametreleri
kernel = np.ones((5, 5), np.uint8)
MAX_AREA_BLUE, MAX_AREA_RED, MIN_AREA = 20000, 20000, 300
alpha_mask = 1
lock_threshold, lost_frame_threshold, position_tolerance = 3, 10, 60

# ----------------- Kamera Soyutlama -----------------
PICAM_AVAILABLE = False
try:
    from picamera2 import Picamera2
    PICAM_AVAILABLE = True
except Exception:
    PICAM_AVAILABLE = False

class FrameGrabber:
    def __init__(self, width=640, height=480, fps=60):
        self.width, self.height, self.fps = width, height, fps
        self.source = None
        self.picam = None
        self.cap = None
        self._open()

    def _open(self):
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
                print("Picamera2 açılamadı, V4L2'ye düşülüyor:", e)

        backend = cv2.CAP_V4L2 if platform.system() != "Windows" else cv2.CAP_DSHOW
        self.cap = cv2.VideoCapture(0, backend)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Kamera açılamadı (ne Picamera2 ne V4L2).")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.source = "opencv"

    def read(self):
        try:
            if self.source == "picamera2":
                arr = self.picam.capture_array()  # RGB888
                frame_bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return True, frame_bgr
            else:
                return self.cap.read()
        except Exception as e:
            print("Kare okunamadı:", e)
            return False, None

    def release(self):
        if self.source == "picamera2" and self.picam is not None:
            try: self.picam.stop()
            except Exception: pass
        if self.source == "opencv" and self.cap is not None:
            self.cap.release()

# ----------------- Geometri & Mesafe -----------------
def angle_cos(p0, p1, p2):
    d1, d2 = p0 - p1, p2 - p1
    return float(np.dot(d1, d2) / (np.linalg.norm(d1)*np.linalg.norm(d2) + 1e-8))

def is_square(c, min_area=400, angle_tol=0.20, aspect_tol=0.20):
    area = cv2.contourArea(c)
    if area < min_area: return False
    approx = cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True)
    if len(approx) != 4 or not cv2.isContourConvex(approx): return False
    pts = approx.reshape(-1,2).astype(np.float32)
    max_cos = max(abs(angle_cos(pts[(i-1)%4], pts[i], pts[(i+1)%4])) for i in range(4))
    if max_cos > angle_tol: return False
    x,y,w,h = cv2.boundingRect(approx)
    if h==0: return False
    if abs((w/float(h))-1.0) > aspect_tol: return False
    return True

def approx_square_corners(contour):
    eps = 0.02 * cv2.arcLength(contour, True)
    ap  = cv2.approxPolyDP(contour, eps, True)
    if ap is None or len(ap) != 4:
        return None
    pts = ap.reshape(-1, 2).astype(np.float32)
    c = np.mean(pts, axis=0)
    ang = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    order = np.argsort(ang)
    pts = pts[order]
    return pts

def side_length_pixels(pts):
    if pts is None or len(pts) != 4: return None
    d01 = np.linalg.norm(pts[0]-pts[1])
    d12 = np.linalg.norm(pts[1]-pts[2])
    d23 = np.linalg.norm(pts[2]-pts[3])
    d30 = np.linalg.norm(pts[3]-pts[0])
    return float((d01 + d12 + d23 + d30) / 4.0)

def ensure_focal_from_fov(frame_w):
    global FOCAL_PX
    if USE_HFOV and FOCAL_PX is None and frame_w > 0:
        FOCAL_PX = (0.5 * frame_w) / math.tan(math.radians(HFOV_DEG * 0.5))

def distance_from_altitude(pixel, frame_shape, H_m):
    h, w = frame_shape[:2]
    ensure_focal_from_fov(w)
    if FOCAL_PX is None or H_m is None:
        return None, None
    u, v = float(pixel[0]), float(pixel[1])
    cx, cy = w/2.0, h/2.0
    f = float(FOCAL_PX)
    x = (u - cx) / f
    y = (v - cy) / f
    ground_range = H_m * math.sqrt(x*x + y*y)
    slant = H_m * math.sqrt(x*x + y*y + 1.0)
    return slant, ground_range

def estimate_distance_m_robust(contour, frame_shape):
    h, w = frame_shape[:2]
    ensure_focal_from_fov(w)
    pts = approx_square_corners(contour)
    if pts is None:
        return None, None, None
    f = FOCAL_PX if FOCAL_PX else 800.0
    cx, cy = w/2.0, h/2.0
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float32)
    dist = np.zeros((5,1), dtype=np.float32)
    s = REAL_SIZE_M / 2.0
    objp = np.array([[-s,-s,0],[s,-s,0],[s,s,0],[-s,s,0]], dtype=np.float32)
    imgp = pts.astype(np.float32)
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    Z = None
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=flags)
    if ok:
        Z = float(np.linalg.norm(tvec))
    else:
        side_px = side_length_pixels(pts)
        if side_px is not None and side_px > 1:
            Z = (f * REAL_SIZE_M) / side_px
    side_px = side_length_pixels(pts)
    return Z, side_px, pts

def bbox_area_px2(contour):
    x, y, w, h = cv2.boundingRect(contour)
    return int(w * h)

def _border_touch_score(contour, frame_shape, margin=3):
    h, w = frame_shape[:2]
    xs = contour[:,:,0]; ys = contour[:,:,1]
    touches = (xs <= margin).any() or (xs >= w-1-margin).any() \
              or (ys <= margin).any() or (ys >= h-1-margin).any()
    return 0.0 if touches else 1.0

def _right_angle_score(pts):
    if pts is None or len(pts)!=4: return 0.0
    angs=[]
    for i in range(4):
        a = pts[(i-1)%4]; b = pts[i]; c = pts[(i+1)%4]
        v1 = a - b; v2 = c - b
        cosang = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)+1e-9)
        cosang = np.clip(cosang, -1, 1)
        deg = np.degrees(np.arccos(cosang))
        angs.append(deg)
    err = np.mean([abs(90.0 - d) for d in angs])
    return float(max(0.0, 1.0 - (err/45.0)))

def _aspect_score_minrect(contour):
    rect = cv2.minAreaRect(contour)
    (w,h) = rect[1]
    if w<=1 or h<=1: return 0.0
    r = max(w,h)/min(w,h)
    return float(max(0.0, 1.0 - (r-1.0)))

def _fill_scores(contour):
    area = max(cv2.contourArea(contour), 1.0)
    x,y,w,h = cv2.boundingRect(contour)
    rect_area = max(w*h, 1.0)
    extent = float(area/rect_area)
    hull = cv2.convexHull(contour)
    hull_area = max(cv2.contourArea(hull), 1.0)
    solidity = float(area/hull_area)
    return extent, solidity

def compute_confidence(t, frame_shape):
    h, w = frame_shape[:2]
    vis = min(t.get("visible_frames", 0) / max(lock_threshold, 1), 1.0)
    c = t.get("contour", None)
    if c is None: return round(float(0.4*vis), 2)
    pts = approx_square_corners(c)
    area = cv2.contourArea(c)
    border = _border_touch_score(c, frame_shape)
    right90 = _right_angle_score(pts)
    aspect = _aspect_score_minrect(c)
    extent, solidity = _fill_scores(c)
    area_norm = max(0.0, min(area/8000.0, 1.0))
    cx, cy = t["cx"], t["cy"]
    offset = math.hypot(cx - w/2, cy - h/2) / (0.5*w)
    offset_score = float(max(0.4, 1.0 - 0.6*offset))
    base = (0.15*vis + 0.20*right90 + 0.15*aspect + 0.20*extent + 0.15*solidity + 0.15*area_norm)
    conf = border * base * offset_score
    soft_cap = 1.0
    if area_norm < 0.3 or right90 < 0.6 or extent < 0.6:
        soft_cap = 0.6
    conf = min(conf, soft_cap)
    return round(float(max(min(conf, 1.0), 0.0)), 2)

def get_current_gps():
    # Buraya gerçek GPS okuma entegre edebilirsiniz
    return None, None

def get_altitude_m():
    # Buraya gerçek irtifa (baro/GNSS) entegre edebilirsiniz
    return None

# ----------------- Asenkron İşçi (UI dışı) -----------------
class ActionWorker(QThread):
    done = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._queue = []
        self._running = True
        self._last_action_ts = 0.0

    def queue_action(self, payload: dict):
        self._queue.append(payload)

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            if not self._queue:
                self.msleep(10)
                continue
            payload = self._queue.pop(0)

            now = time.time()
            if now - self._last_action_ts < ACTION_COOLDOWN_SEC:
                continue
            self._last_action_ts = now

            try:
                mavlink_func.test_mavlink_connection()
            except Exception as e:
                self.done.emit(f"MAVLink hata: {e}")
                continue

            try:
                controller.guided_approach_velocity(master,)
                self.done.emit("motion.planning() OK")
            except Exception as e:
                self.done.emit(f"motion.planning hata: {e}")

# ----------------- Algılama -----------------
def detect_targets(mask, min_area, tracked_list):
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current = []
    for c in contours:
        if cv2.contourArea(c) >= min_area and is_square(c, MIN_AREA):
            M = cv2.moments(c)
            if M["m00"]!=0:
                cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                current.append({"cx":cx,"cy":cy,"locked":False,"action_sent":False,
                                "visible_frames":1,"lost_frames":0,"contour":c})

    # Basit yakınlık eşleştirme
    for det in current:
        matched = False
        for t in tracked_list:
            if abs(det["cx"]-t["cx"])<position_tolerance and abs(det["cy"]-t["cy"])<position_tolerance:
                t.update({"cx":det["cx"],"cy":det["cy"]})
                t["contour"] = det["contour"]
                t["visible_frames"]+=1; t["lost_frames"]=0
                matched=True; break
        if not matched: tracked_list.append(det)

    for t in tracked_list:
        if all(abs(t["cx"]-d["cx"])>position_tolerance or abs(t["cy"]-d["cy"])>position_tolerance for d in current):
            t["lost_frames"]+=1
    tracked_list[:]=[t for t in tracked_list if t["lost_frames"]<=lost_frame_threshold]


# ------ state manger -----

def build_detection_info(color_name, target_dict, frame_shape,state: StateManager):
    cx, cy = int(target_dict["cx"]), int(target_dict["cy"])
    contour = target_dict.get("contour", None)
    area_px2 = bbox_area_px2(contour) if contour is not None else 0
    H = get_altitude_m() if USE_ALTITUDE else None
    dist_alt, ground_range = (None, None)
    if H is not None:
        dist_alt, ground_range = distance_from_altitude((cx, cy), frame_shape, H)
    dist_size, side_px, pts = (None, None, None)
    if dist_alt is None and contour is not None:
        dist_size, side_px, pts = estimate_distance_m_robust(contour, frame_shape)
    chosen_dist = dist_alt if dist_alt is not None else dist_size
    lat, lon = get_current_gps()
    state.update_target(color_name, cx, cy, int(area_px2), compute_confidence(target_dict, frame_shape))
    info = {
        "detected": True,
        "type": f"{color_name}_4x4",
        "pixel": [cx, cy],
        "bbox_area": int(area_px2),
        "distance_m": round(float(chosen_dist), 2) if chosen_dist is not None else None,
        "target_lat": float(lat) if lat is not None else None,
        "target_lon": float(lon) if lon is not None else None,
        "confidence": compute_confidence(target_dict, frame_shape)
    }
    debug = {
        "side_px": side_px,
        "FOCAL_PX": FOCAL_PX,
        "HFOV_DEG": HFOV_DEG,
        "altitude_m": H,
        "ground_range_m": ground_range,
        "pnp_pts": pts
    }
    return info, debug

# ----------------- PySide6 Arayüz -----------------
class IHAInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("İHA Görüntü ve TeleMETRİ Arayüzü (RPi uyumlu)")
        self.setGeometry(40, 40, 1700, 900)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        title = QLabel("🚁 İHA GÖRÜNTÜ VE TELEMETRİ ARAYÜZÜ")
        title.setFont(QFont("Arial", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        self.label_main = self.make_video_frame("Ana Görsel")
        self.label_blue = self.make_video_frame("Mavi Maske (Kare)")
        self.label_red  = self.make_video_frame("Kırmızı Maske (Kare)")

        video_layout = QHBoxLayout()
        video_layout.addWidget(self.label_main)
        video_layout.addWidget(self.label_blue)
        video_layout.addWidget(self.label_red)

        self.status_label = QLabel("Durum: Hazır")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setAlignment(Qt.AlignLeft)

        self.json_label = QLabel("{}")
        self.json_label.setFont(QFont("Consolas", 10))
        self.json_label.setAlignment(Qt.AlignLeft)
        self.json_label.setStyleSheet("background:#111; border:1px solid #444; padding:6px;")

        bottom_layout = QHBoxLayout()
        left_box = QVBoxLayout()
        left_box.addWidget(self.status_label)
        left_box.addWidget(self.json_label)
        bottom_layout.addLayout(left_box)
        bottom_layout.addStretch()

        btn_start = QPushButton("▶ Başlat")
        btn_stop  = QPushButton("⏹ Durdur")
        btn_exit  = QPushButton("❌ Çıkış")
        for b in (btn_start, btn_stop, btn_exit):
            b.setFixedHeight(40)
            b.setStyleSheet("""
                QPushButton { background-color: #2e8b57; color: white; font-size: 14px; border-radius: 8px; }
                QPushButton:hover { background-color: #3cb371; }
            """)

        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(btn_start)
        button_layout.addWidget(btn_stop)
        button_layout.addWidget(btn_exit)

        main_layout = QVBoxLayout()
        main_layout.addWidget(title)
        main_layout.addLayout(video_layout)
        main_layout.addLayout(bottom_layout)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)

        btn_start.clicked.connect(self.start_stream)
        btn_stop.clicked.connect(self.stop_stream)
        btn_exit.clicked.connect(self.close)

        # Durum/Debug
        self.last_detection_json = {}
        self.debug = {}
        self.print_counter = 0

        # Frame kaynağı
        self.grabber = FrameGrabber(width=1280, height=720, fps=30)

        # UI dışı işçi
        self.worker = ActionWorker()
        self.worker.done.connect(self.on_worker_done)
        self.worker.start()

        # State manager
        self.state_manager = StateManager()

        # Maske belleği ve takip listeleri
        self.prev_mask_blue, self.prev_mask_red = None, None
        self.tracked_blue, self.tracked_red = [], []

    def make_video_frame(self, title):
        frame = QLabel(title)
        frame.setFixedSize(500, 400)
        frame.setStyleSheet("background-color: black; border: 2px solid gray; color: white;")
        frame.setAlignment(Qt.AlignCenter)
        return frame

    def start_stream(self):
        self.status_label.setText("Durum: Yayın Başladı")
        self.timer.start(33)  # ~30 fps

    def stop_stream(self):
        self.status_label.setText("Durum: Durduruldu")
        self.timer.stop()

    def closeEvent(self, e):
        try:
            self.timer.stop()
            self.worker.stop()
            self.worker.wait(500)
            self.grabber.release()
        except Exception:
            pass
        return super().closeEvent(e)

    def set_frame(self,label,frame):
        rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        h,w,ch = rgb.shape
        qimg = QImage(rgb.data,w,h,ch*w,QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(label.width(),label.height(),Qt.KeepAspectRatio)
        label.setPixmap(pix)

    def on_worker_done(self, msg: str):
        self.status_label.setText(f"Durum: {msg}")

    def draw_debug_overlay(self, img):
        y = 24
        def put(line):
            nonlocal y
            cv2.putText(img, line, (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
            y += 22
        put(f"HFOV(deg): {HFOV_DEG}")
        put(f"FOCAL_PX : {FOCAL_PX:.2f}" if FOCAL_PX else "FOCAL_PX : (auto after first frame)")
        put(f"REAL_SIZE : {REAL_SIZE_M*100:.1f} cm")
        put(f"ALT_MODE  : {'ON' if USE_ALTITUDE else 'OFF'}")
        if self.debug:
            if self.debug.get("altitude_m") is not None:
                put(f"altitude : {self.debug['altitude_m']:.2f} m")
            if self.debug.get("ground_range_m") is not None:
                put(f"groundR  : {self.debug['ground_range_m']:.2f} m")
            if self.debug.get("side_px") is not None:
                put(f"side_px  : {self.debug['side_px']:.2f}")

    def update_frames(self):
        ret, frame_bgr = self.grabber.read()
        if not ret or frame_bgr is None:
            self.status_label.setText("Durum: Kare okunamadı")
            return

        ensure_focal_from_fov(frame_bgr.shape[1])

        # Aynalama
        frame_bgr = cv2.flip(frame_bgr, 1)

        # Çok karanlıksa algılama atla
        hsv_for_check = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        if hsv_for_check[...,2].mean() < 10:
            disp_main = frame_bgr.copy()
            cv2.putText(disp_main, "NO IMAGE (low brightness) - skipping detection",
                        (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)
            self.set_frame(self.label_main, disp_main)
            self.set_frame(self.label_blue, np.zeros_like(frame_bgr))
            self.set_frame(self.label_red,  np.zeros_like(frame_bgr))
            return

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        ref_x, ref_y = frame_bgr.shape[1]//2, frame_bgr.shape[0]//2

        disp_main = frame_bgr.copy()
        cv2.circle(disp_main,(ref_x,ref_y),6,(0,255,255),-1)

        # HSV eşikleri (ihtiyaca göre ayarlayın)
        mask_blue = cv2.inRange(hsv,(90,80,50),(130,255,255))
        mask_red  = cv2.inRange(hsv,(0,180,150),(10,255,255)) | cv2.inRange(hsv,(160,100,100),(179,255,255))

        # Morfoloji
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel,1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel,2)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_OPEN, kernel,1)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_CLOSE, kernel,2)

        # Zamanla yumuşatma
        if self.prev_mask_blue is not None:
            mask_blue = cv2.addWeighted(mask_blue, alpha_mask, self.prev_mask_blue, 1-alpha_mask, 0).astype(np.uint8)
        self.prev_mask_blue = mask_blue.copy()
        if self.prev_mask_red is not None:
            mask_red = cv2.addWeighted(mask_red, alpha_mask, self.prev_mask_red, 1-alpha_mask, 0).astype(np.uint8)
        self.prev_mask_red = mask_red.copy()

        # Takip/algılama
        detect_targets(mask_blue, MIN_AREA, self.tracked_blue)
        detect_targets(mask_red,  1000,     self.tracked_red)

        disp_blue = cv2.cvtColor(mask_blue,cv2.COLOR_GRAY2BGR)
        disp_red  = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)

        detection_json_to_show = None
        self.debug = {}

        for color_name, tracked_list, disp, color in [
            ("blue", self.tracked_blue, disp_blue, (255,0,0)),
            ("red",  self.tracked_red,  disp_red,  (0,0,255))
        ]:
            cv2.circle(disp,(ref_x,ref_y),6,(0,255,255),-1)
            for t in tracked_list:
                if not t["locked"] and t["visible_frames"]>=lock_threshold:
                    t["locked"]=True

                if t["locked"]:
                    # Çizimler
                    if t.get("contour") is not None:
                        cv2.drawContours(disp,[t["contour"]],-1,(0,0,255),2)
                    cv2.circle(disp,(t["cx"],t["cy"]),6,(0,0,255),-1)

                    # Bilgi
                    info, dbg = build_detection_info(color_name, t, frame_bgr.shape, self.state_manager)
                    detection_json_to_show = info
                    self.debug = dbg

                    # Ağır işleri worker'a ver (tek sefer, cooldown ile)
                    if not t.get("action_sent", False):
                        self.worker.queue_action({
                            "color": color_name,
                            "pixel": (t["cx"], t["cy"]),
                            "info": info
                        })
                        t["action_sent"] = True

                    # Konsol çıktısını seyrekleştir
                    self.print_counter += 1
                    if self.print_counter % PRINT_EVERY_N == 0:
                        try:
                            print(json.dumps(info, ensure_ascii=False))
                        except Exception:
                            pass
                else:
                    # Henüz kilit yoksa küçük nokta
                    cv2.circle(disp,(t["cx"],t["cy"]),4,color,-1)

        # Overlay & UI güncelle
        self.draw_debug_overlay(disp_main)
        self.set_frame(self.label_main,disp_main)
        self.set_frame(self.label_blue,disp_blue)
        self.set_frame(self.label_red,disp_red)

        if detection_json_to_show is not None:
            self.last_detection_json = detection_json_to_show
            self.json_label.setText(json.dumps(detection_json_to_show, ensure_ascii=False))

# ----------------- Çalıştır -----------------
if __name__=="__main__":
    app = QApplication(sys.argv)
    win = IHAInterface()
    win.show()
    sys.exit(app.exec())

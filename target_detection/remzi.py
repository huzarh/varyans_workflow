# cammod3_picam_gui.py — RPi Camera Module 3 (IMX708) + Picamera2 + PySide6
import sys
import cv2
import numpy as np
import math
import json 
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout
)
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtCore import Qt, QTimer
from pymavlink import mavutil
from mavlink_func
import time


mavlink_func.test_mavlink_connection()
# print("MAVLink bağlantısı kuruluyor...")

# try:
#     master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
#     master.wait_heartbeat(timeout=5)
#     print(f"✅ MAVLink bağlandı! System: {master.target_system}, Component: {master.target_component}")
# except Exception as e:
#     print(f"❌ MAVLink bağlantısı başarısız: {e}")
#     sys.exit(1)

# # ------------------ Test Komutu Gönder ------------------
# try:
#     print("📤 TEST: STATUSTEXT mesajı gönderiliyor...")
#     master.mav.statustext_send(
#         mavutil.mavlink.MAV_SEVERITY_NOTICE,
#         b"Baglanti testi: Varyans sistem basladi."
#     )
#     print("✅ STATUSTEXT mesajı gönderildi.")
# except Exception as e:
#     print(f"❌ MAVLink mesaj gönderimi hatası: {e}")
#     sys.exit(1)


# ----------------- PiCamera2 İçe Aktarım -----------------
from picamera2 import Picamera2

# ----------------- OpenCV / Algılama Ayarları -----------------
kernel = np.ones((5, 5), np.uint8)

MAX_AREA_BLUE, MAX_AREA_RED, MIN_AREA = 20000, 20000, 300
alpha_mask = 1.0
prev_mask_blue, prev_mask_red = None, None
tracked_blue, tracked_red = [], []
lock_threshold, lost_frame_threshold, position_tolerance = 3, 10, 60

# ------- MESAFE / KALİBRASYON AYARLARI -------
REAL_SIZE_M = 0.05   # 5 cm kare hedef
USE_HFOV   = True    # HFOV'tan odak hesapla
HFOV_DEG   = 66.0    # Standart lens ~66°; wide lens ~120°
FOCAL_PX   = None    # USE_HFOV=True iken otomatik hesaplanır; sabitlemek istersen sayı ver

# ------- ALTITUDE (Boş Alan) TABANLI MESAFE ------
USE_ALTITUDE = True  # True ise, H (irtifa) varsa öncelikle bu yöntem kullanılır

# ----------------- Geometri Yardımcıları -----------------
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

# --------- ALTITUDE tabanlı mesafe ----------
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

# --------- Boyut temelli (PnP / yedek) ----------
def estimate_distance_m_robust(contour, frame_shape):
    h, w = frame_shape[:2]
    ensure_focal_from_fov(w)

    pts = approx_square_corners(contour)
    if pts is None:
        return None, None, None

    f = FOCAL_PX if FOCAL_PX else 800.0
    cx, cy = w/2.0, h/2.0
    K = np.array([[f, 0, cx],
                  [0, f, cy],
                  [0, 0,  1]], dtype=np.float32)
    dist = np.zeros((5,1), dtype=np.float32)

    s = REAL_SIZE_M / 2.0
    objp = np.array([[-s, -s, 0],
                     [ s, -s, 0],
                     [ s,  s, 0],
                     [-s,  s, 0]], dtype=np.float32)
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

# ----------------- GELİŞMİŞ CONFIDENCE -----------------
def _border_touch_score(contour, frame_shape, margin=3):
    h, w = frame_shape[:2]
    xs = contour[:,:,0]
    ys = contour[:,:,1]
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
    if c is None:
        return round(float(0.4*vis), 2)

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

    base = (
        0.15*vis +
        0.20*right90 +
        0.15*aspect +
        0.20*extent +
        0.15*solidity +
        0.15*area_norm
    )

    conf = border * base * offset_score
    soft_cap = 1.0
    if area_norm < 0.3 or right90 < 0.6 or extent < 0.6:
        soft_cap = 0.6
    conf = min(conf, soft_cap)

    return round(float(max(min(conf, 1.0), 0.0)), 2)

# ----------------- SENSÖR KANCALARI -----------------
def get_current_gps():
    return None, None  # MAVLink/GPSD ile doldur

def get_altitude_m():
    return None        # Örn. 10.0 verip test edebilirsin

# ----------------- Tespit & Takip -----------------
def detect_targets(mask, min_area, tracked_list):
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current = []
    for c in contours:
        if cv2.contourArea(c) >= min_area and is_square(c, MIN_AREA):
            M = cv2.moments(c)
            if M["m00"]!=0:
                cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                current.append({"cx":cx,"cy":cy,"locked":False,
                                "visible_frames":1,"lost_frames":0,"contour":c})
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
    tracked_list[:] = [t for t in tracked_list if t["lost_frames"]<=lost_frame_threshold]

def build_detection_info(color_name, target_dict, frame_shape):
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

# ----------------- PiCamera2 Sarmalayıcı -----------------
class PiCam:
    def __init__(self, size=(1280, 720), flip_h=True):
        self.picam = Picamera2()
        self.flip_h = flip_h
        # RGB888 formatıyla ana stream; hızlı ve OpenCV ile uyumlu
        config = self.picam.create_video_configuration(
            main={"size": size, "format": "BGR888"}
        )
        self.picam.configure(config)
        # Otomatik kontrolleri varsayılan bırak (AE/AWB/AF yok; sabit fokus lens)
        self.picam.start()

    def read(self):
        """
        OpenCV uyumlu BGR frame döndürür.
        """
        rgb = self.picam.capture_array()  # RGB
        if self.flip_h:
            rgb = np.ascontiguousarray(np.fliplr(rgb))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return True, bgr

    def stop(self):
        try:
            self.picam.stop()
        except Exception:
            pass

# ----------------- PySide6 Arayüz -----------------
class IHAInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("İHA Görüntü ve TeleMETRİ Arayüzü")
        self.setGeometry(100, 100, 1700, 900)
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

        logo = QLabel()
        pixmap = QPixmap("Varyans logo.jpg")
        if not pixmap.isNull():
            logo.setPixmap(pixmap.scaled(200, 150, Qt.KeepAspectRatio))
        logo.setAlignment(Qt.AlignRight)

        bottom_layout = QHBoxLayout()
        left_box = QVBoxLayout()
        left_box.addWidget(self.status_label)
        left_box.addWidget(self.json_label)
        bottom_layout.addLayout(left_box)
        bottom_layout.addStretch()
        bottom_layout.addWidget(logo)

        btn_start = QPushButton("▶ Başlat")
        btn_stop  = QPushButton("⏹ Durdur")
        btn_exit  = QPushButton("❌ Çıkış")
        for b in (btn_start, btn_stop, btn_exit):
            b.setFixedHeight(40)
            b.setStyleSheet("""
                QPushButton {
                    background-color: #2e8b57;
                    color: white;
                    font-size: 14px;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #3cb371;
                }
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

        # --- PiCamera2 başlat ---
        self.cam = PiCam(size=(1280, 720), flip_h=True)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)

        btn_start.clicked.connect(self.start_stream)
        btn_stop.clicked.connect(self.stop_stream)
        btn_exit.clicked.connect(self.close)

        self.last_detection_json = {}
        self.debug = {}

    def make_video_frame(self, title):
        frame = QLabel(title)
        frame.setFixedSize(500, 400)
        frame.setStyleSheet("background-color: black; border: 2px solid gray; color: white;")
        frame.setAlignment(Qt.AlignCenter)
        return frame

    def start_stream(self):
        self.status_label.setText("Durum: Yayın Başladı")
        self.timer.start(30)  # ~33 FPS hedef

    def stop_stream(self):
        self.status_label.setText("Durum: Durduruldu")
        self.timer.stop()

    def closeEvent(self, event):
        # Kamera temiz kapatılsın
        try:
            self.timer.stop()
        except Exception:
            pass
        try:
            self.cam.stop()
        except Exception:
            pass
        return super().closeEvent(event)

    def update_frames(self):
        global prev_mask_blue, prev_mask_red
        ret, frame = self.cam.read()
        if not ret:
            return

        ensure_focal_from_fov(frame.shape[1])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ref_x, ref_y = frame.shape[1]//2, frame.shape[0]//2

        disp_main = frame.copy()
        cv2.circle(disp_main,(ref_x,ref_y),6,(0,255,255),-1)

        # HSV eşikleri (sahada gerektiğinde ayarla)
        mask_blue = cv2.inRange(hsv,(90,80,50),(130,255,255))
        mask_red  = cv2.inRange(hsv,(0,180,150),(10,255,255)) | cv2.inRange(hsv,(160,100,100),(179,255,255))

        # Morfolojik temizlik
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel,1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel,2)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_OPEN, kernel,1)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_CLOSE, kernel,2)

        # Zaman ağırlıklı yumuşatma
        if prev_mask_blue is not None:
            mask_blue = cv2.addWeighted(mask_blue,alpha_mask,prev_mask_blue,1-alpha_mask,0).astype(np.uint8)
        prev_mask_blue = mask_blue.copy()
        if prev_mask_red is not None:
            mask_red  = cv2.addWeighted(mask_red, alpha_mask,prev_mask_red, 1-alpha_mask,0).astype(np.uint8)
        prev_mask_red = mask_red.copy()

        # Takip
        detect_targets(mask_blue, MIN_AREA, tracked_blue)
        detect_targets(mask_red,  1000,     tracked_red)

        disp_blue = cv2.cvtColor(mask_blue,cv2.COLOR_GRAY2BGR)
        disp_red  = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)

        detection_json_to_show = None
        self.debug = {}

        for color_name, tracked_list, disp, color in [("blue", tracked_blue, disp_blue, (255,0,0)),
                                                      ("red",  tracked_red,  disp_red,  (0,0,255))]:
            cv2.circle(disp,(ref_x,ref_y),6,(0,255,255),-1)
            for t in tracked_list:
                if not t["locked"] and t["visible_frames"]>=lock_threshold:
                    t["locked"]=True
                if t["locked"]:
                    cv2.drawContours(disp,[t["contour"]],-1,(0,0,255),2)
                    cv2.circle(disp,(t["cx"],t["cy"]),6,(0,0,255),-1)

                    info, dbg = build_detection_info(color_name, t, frame.shape)
                    send_mode
                    detection_json_to_show = info
                    self.debug = dbg

                    print(json.dumps(info, ensure_ascii=False))
                else:
                    cv2.circle(disp,(t["cx"],t["cy"]),4,color,-1)

        # Debug overlay
        self.draw_debug_overlay(disp_main)

        self.set_frame(self.label_main, disp_main)
        self.set_frame(self.label_blue, disp_blue)
        self.set_frame(self.label_red,  disp_red)

        if detection_json_to_show is not None:
            self.last_detection_json = detection_json_to_show
            self.json_label.setText(json.dumps(detection_json_to_show, ensure_ascii=False))

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

    def set_frame(self,label,frame):
        rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        h,w,ch = rgb.shape
        qimg = QImage(rgb.data,w,h,ch*w,QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(label.width(),label.height(),Qt.KeepAspectRatio)
        label.setPixmap(pix)

# ----------------- Çalıştır -----------------
if __name__=="__main__":
    app = QApplication(sys.argv)
    win = IHAInterface()
    win.show()
    sys.exit(app.exec())

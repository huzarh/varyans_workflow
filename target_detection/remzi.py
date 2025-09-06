import sys
import cv2
import numpy as np
import math
import json
from picamera2 import Picamera2
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout
)
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtCore import Qt, QTimer

# ----------------- Kamera (Picamera2) -----------------
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

kernel = np.ones((5, 5), np.uint8)

MAX_AREA_BLUE, MAX_AREA_RED, MIN_AREA = 20000, 20000, 300
alpha_mask = 1
prev_mask_blue, prev_mask_red = None, None
tracked_blue, tracked_red = [], []
lock_threshold, lost_frame_threshold, position_tolerance = 3, 10, 60

# ------- MESAFE / KALİBRASYON AYARLARI -------
REAL_SIZE_M = 0.05   # 5 cm kare hedef
USE_HFOV   = True
HFOV_DEG   = 66.0
FOCAL_PX   = None

USE_ALTITUDE = True

# ----------------- Geometri ve diğer fonksiyonlar -----------------
# (Senin yazdığın tüm fonksiyonlar aynı kalabilir, sadece frame kaynağı değişti)

def ensure_focal_from_fov(frame_w):
    global FOCAL_PX
    if USE_HFOV and FOCAL_PX is None and frame_w > 0:
        FOCAL_PX = (0.5 * frame_w) / math.tan(math.radians(HFOV_DEG * 0.5))

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

    def make_video_frame(self, title):
        frame = QLabel(title)
        frame.setFixedSize(500, 400)
        frame.setStyleSheet("background-color: black; border: 2px solid gray; color: white;")
        frame.setAlignment(Qt.AlignCenter)
        return frame

    def start_stream(self):
        self.status_label.setText("Durum: Yayın Başladı")
        self.timer.start(30)

    def stop_stream(self):
        self.status_label.setText("Durum: Durduruldu")
        self.timer.stop()

    def update_frames(self):
        global prev_mask_blue, prev_mask_red

        frame = picam2.capture_array()   # <-- Webcam yerine Picamera2’den alıyoruz
        if frame is None:
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Picamera2 RGB verir
        frame = cv2.flip(frame, 1)

        ensure_focal_from_fov(frame.shape[1])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv,(90,80,50),(130,255,255))
        mask_red  = cv2.inRange(hsv,(0,180,150),(10,255,255)) | cv2.inRange(hsv,(160,100,100),(179,255,255))

        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, 1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, 2)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_OPEN, kernel, 1)
        mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_CLOSE, kernel, 2)

        disp_main = frame.copy()
        disp_blue = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
        disp_red  = cv2.cvtColor(mask_red,  cv2.COLOR_GRAY2BGR)

        self.set_frame(self.label_main, disp_main)
        self.set_frame(self.label_blue, disp_blue)
        self.set_frame(self.label_red, disp_red)

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
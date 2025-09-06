#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
İHA Hedef Tespit + MAVLink GUIDED Mod Entegre Sistem
Remzi.py + switch_to_guided.py entegrasyonu
Hedef tespit edildiğinde otomatik GUIDED moda geçiş
"""

import sys
import cv2
import numpy as np
import math
import json
import time
import threading
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QCheckBox
)
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtCore import Qt, QTimer, pyqtSignal

# MAVLink import
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("⚠️ MAVLink kütüphanesi bulunamadı! pip install pymavlink")

# PiCamera2 import
from picamera2 import Picamera2

# ================== MAVLink Yöneticisi ==================
class MAVLinkManager:
    """MAVLink bağlantı ve komut yöneticisi"""
    
    def __init__(self, connection_string='udp:127.0.0.1:14540'):
        self.connection_string = connection_string
        self.master = None
        self.connected = False
        self.target_detected = False
        self.guided_mode_set = False
        
    def connect(self):
        """MAVLink bağlantısı kur"""
        if not MAVLINK_AVAILABLE:
            print("❌ MAVLink kütüphanesi yüklü değil!")
            return False
            
        try:
            print(f"🔗 MAVLink bağlantısı kuruluyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat(timeout=5)
            self.connected = True
            print(f"✅ MAVLink bağlandı! System: {self.master.target_system}, Component: {self.master.target_component}")
            return True
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            self.connected = False
            return False
    
    def set_mode(self, mode_id, mode_name):
        """İHA modunu değiştir"""
        if not self.connected or not self.master:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        try:
            print(f"🔄 {mode_name} moduna geçiliyor...")
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Mod değişikliğini doğrula
            time.sleep(2)
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if hb and hb.custom_mode == mode_id:
                print(f"✅ {mode_name} moduna geçildi")
                return True
            else:
                print(f"❌ {mode_name} moduna geçilemedi")
                return False
        except Exception as e:
            print(f"❌ Mod değiştirme hatası: {e}")
            return False
    
    def switch_to_guided(self):
        """GUIDED moda geç"""
        return self.set_mode(15, "GUIDED")
    
    def switch_to_auto(self):
        """AUTO moda geç"""
        return self.set_mode(3, "AUTO")
    
    def get_current_mode(self):
        """Mevcut modu al"""
        if not self.connected or not self.master:
            return None, "Bağlantı Yok"
            
        try:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
            if hb:
                mode_mapping = {
                    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
                    4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
                    15: "GUIDED", 16: "POSITION"
                }
                return hb.custom_mode, mode_mapping.get(hb.custom_mode, f"MODE_{hb.custom_mode}")
        except Exception as e:
            print(f"Mod okuma hatası: {e}")
            
        return None, "Bilinmiyor"
    
    def on_target_detected(self):
        """Hedef tespit edildiğinde çağrılır"""
        if not self.target_detected and self.connected:
            self.target_detected = True
            print("🎯 HEDEF TESPİT EDİLDİ!")
            
            # GUIDED moda geç
            if not self.guided_mode_set:
                success = self.switch_to_guided()
                if success:
                    self.guided_mode_set = True
                    print("🚁 İHA GUIDED modda - Müdahale için hazır!")
                else:
                    print("⚠️ GUIDED moda geçilemedi!")
    
    def reset_target_status(self):
        """Hedef durumunu sıfırla"""
        self.target_detected = False
        self.guided_mode_set = False
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.master:
            try:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")
            except:
                pass
        self.connected = False

# ================== Hedef Tespit Sistemi ==================

# OpenCV / Algılama Ayarları
kernel = np.ones((5, 5), np.uint8)
MAX_AREA_BLUE, MAX_AREA_RED, MIN_AREA = 20000, 20000, 300
alpha_mask = 1.0
prev_mask_blue, prev_mask_red = None, None
tracked_blue, tracked_red = [], []
lock_threshold, lost_frame_threshold, position_tolerance = 3, 10, 60

# Mesafe / Kalibrasyon Ayarları
REAL_SIZE_M = 0.05
USE_HFOV = True
HFOV_DEG = 66.0
FOCAL_PX = None
USE_ALTITUDE = True

# Geometri Yardımcıları
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

def ensure_focal_from_fov(frame_w):
    global FOCAL_PX
    if USE_HFOV and FOCAL_PX is None and frame_w > 0:
        FOCAL_PX = (0.5 * frame_w) / math.tan(math.radians(HFOV_DEG * 0.5))

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

# PiCamera2 Sarmalayıcı
class PiCam:
    def __init__(self, size=(1280, 720), flip_h=True):
        self.picam = Picamera2()
        self.flip_h = flip_h
        config = self.picam.create_video_configuration(
            main={"size": size, "format": "BGR888"}
        )
        self.picam.configure(config)
        self.picam.start()

    def read(self):
        try:
            rgb = self.picam.capture_array()
            if self.flip_h:
                rgb = np.ascontiguousarray(np.fliplr(rgb))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return True, bgr
        except:
            return False, None

    def stop(self):
        try:
            self.picam.stop()
        except Exception:
            pass

# ================== Ana Arayüz ==================
class IntegratedIHAInterface(QWidget):
    """Entegre İHA Arayüzü - Hedef Tespit + MAVLink"""
    
    def __init__(self):
        super().__init__()
        
        # MAVLink Yöneticisi
        self.mavlink_manager = MAVLinkManager()
        
        self.setup_ui()
        self.setup_camera()
        
        # Durum değişkenleri
        self.last_detection_json = {}
        self.debug = {}
        self.targets_detected_count = 0
        
    def setup_ui(self):
        """Arayüz kurulumu"""
        self.setWindowTitle("🚁 İHA Hedef Tespit + MAVLink Entegre Sistem")
        self.setGeometry(100, 100, 1600, 1000)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        # Başlık
        title = QLabel("🚁 İHA HEDEF TESPİT + MAVLink ENTEGRE SİSTEM")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #00ff00; margin: 10px;")

        # Video görüntüleri
        self.label_main = self.make_video_frame("Ana Görsel")
        self.label_blue = self.make_video_frame("Mavi Hedefler")
        self.label_red = self.make_video_frame("Kırmızı Hedefler")

        video_layout = QHBoxLayout()
        video_layout.addWidget(self.label_main)
        video_layout.addWidget(self.label_blue)
        video_layout.addWidget(self.label_red)

        # MAVLink durumu
        self.mavlink_status = QLabel("MAVLink: Bağlı Değil")
        self.mavlink_status.setFont(QFont("Arial", 12, QFont.Bold))
        self.mavlink_status.setStyleSheet("color: red; padding: 5px;")
        
        self.current_mode = QLabel("Mod: Bilinmiyor")
        self.current_mode.setFont(QFont("Arial", 12))
        
        # Tespit durumu
        self.detection_status = QLabel("Hedef Durumu: Tespit Edilmedi")
        self.detection_status.setFont(QFont("Arial", 12, QFont.Bold))
        self.detection_status.setStyleSheet("color: orange; padding: 5px;")

        # Otomatik GUIDED mod seçeneği
        self.auto_guided_checkbox = QCheckBox("Hedef tespit edildiğinde otomatik GUIDED moda geç")
        self.auto_guided_checkbox.setChecked(True)
        self.auto_guided_checkbox.setStyleSheet("color: white; font-size: 12px;")

        # Sistem durumu
        self.status_label = QLabel("Durum: Hazır")
        self.status_label.setFont(QFont("Arial", 12))

        # JSON çıktı
        self.json_label = QLabel("{}")
        self.json_label.setFont(QFont("Consolas", 10))
        self.json_label.setStyleSheet("background:#111; border:1px solid #444; padding:6px; max-height: 100px;")

        # Kontrol butonları
        self.btn_connect_mavlink = QPushButton("🔗 MAVLink Bağlan")
        self.btn_start = QPushButton("▶ Tespit Başlat")
        self.btn_stop = QPushButton("⏹ Durdur")
        self.btn_guided = QPushButton("🚁 GUIDED Mod")
        self.btn_auto = QPushButton("🛫 AUTO Mod")
        self.btn_reset = QPushButton("🔄 Sıfırla")
        self.btn_exit = QPushButton("❌ Çıkış")

        # Buton stilleri
        for btn in [self.btn_connect_mavlink, self.btn_start, self.btn_stop, 
                    self.btn_guided, self.btn_auto, self.btn_reset, self.btn_exit]:
            btn.setFixedHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2e8b57;
                    color: white;
                    font-size: 12px;
                    font-weight: bold;
                    border-radius: 8px;
                    padding: 5px;
                }
                QPushButton:hover {
                    background-color: #3cb371;
                }
                QPushButton:pressed {
                    background-color: #228b22;
                }
            """)

        # Layout düzenleme
        status_layout = QHBoxLayout()
        status_layout.addWidget(self.mavlink_status)
        status_layout.addWidget(self.current_mode)
        status_layout.addStretch()
        status_layout.addWidget(self.detection_status)

        button_layout1 = QHBoxLayout()
        button_layout1.addWidget(self.btn_connect_mavlink)
        button_layout1.addWidget(self.btn_start)
        button_layout1.addWidget(self.btn_stop)
        button_layout1.addStretch()

        button_layout2 = QHBoxLayout()
        button_layout2.addWidget(self.btn_guided)
        button_layout2.addWidget(self.btn_auto)
        button_layout2.addWidget(self.btn_reset)
        button_layout2.addStretch()
        button_layout2.addWidget(self.btn_exit)

        info_layout = QVBoxLayout()
        info_layout.addWidget(self.status_label)
        info_layout.addWidget(self.auto_guided_checkbox)
        info_layout.addWidget(self.json_label)

        main_layout = QVBoxLayout()
        main_layout.addWidget(title)
        main_layout.addLayout(video_layout)
        main_layout.addLayout(status_layout)
        main_layout.addLayout(button_layout1)
        main_layout.addLayout(button_layout2)
        main_layout.addLayout(info_layout)

        self.setLayout(main_layout)

        # Sinyal bağlantıları
        self.btn_connect_mavlink.clicked.connect(self.connect_mavlink)
        self.btn_start.clicked.connect(self.start_detection)
        self.btn_stop.clicked.connect(self.stop_detection)
        self.btn_guided.clicked.connect(self.manual_guided_mode)
        self.btn_auto.clicked.connect(self.manual_auto_mode)
        self.btn_reset.clicked.connect(self.reset_system)
        self.btn_exit.clicked.connect(self.close)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)
        
        # MAVLink durum timer
        self.mavlink_timer = QTimer()
        self.mavlink_timer.timeout.connect(self.update_mavlink_status)
        self.mavlink_timer.start(1000)  # Her saniye güncelle

    def make_video_frame(self, title):
        frame = QLabel(title)
        frame.setFixedSize(500, 400)
        frame.setStyleSheet("background-color: black; border: 2px solid gray; color: white;")
        frame.setAlignment(Qt.AlignCenter)
        return frame

    def setup_camera(self):
        """Kamera kurulumu"""
        try:
            self.cam = PiCam(size=(1280, 720), flip_h=True)
            self.status_label.setText("Durum: Kamera hazır")
        except Exception as e:
            self.status_label.setText(f"Durum: Kamera hatası - {e}")
            self.cam = None

    def connect_mavlink(self):
        """MAVLink bağlantısı kur"""
        if self.mavlink_manager.connect():
            self.mavlink_status.setText("MAVLink: Bağlı ✅")
            self.mavlink_status.setStyleSheet("color: green; padding: 5px;")
            self.btn_connect_mavlink.setText("🔗 Bağlı")
            self.btn_connect_mavlink.setEnabled(False)
        else:
            self.mavlink_status.setText("MAVLink: Bağlantı Hatası ❌")
            self.mavlink_status.setStyleSheet("color: red; padding: 5px;")

    def update_mavlink_status(self):
        """MAVLink durumunu güncelle"""
        if self.mavlink_manager.connected:
            mode_id, mode_name = self.mavlink_manager.get_current_mode()
            if mode_name:
                self.current_mode.setText(f"Mod: {mode_name}")
                if mode_name == "GUIDED":
                    self.current_mode.setStyleSheet("color: lime; font-weight: bold;")
                elif mode_name == "AUTO":
                    self.current_mode.setStyleSheet("color: yellow; font-weight: bold;")
                else:
                    self.current_mode.setStyleSheet("color: white;")

    def start_detection(self):
        """Hedef tespit başlat"""
        if self.cam:
            self.status_label.setText("Durum: Tespit aktif")
            self.timer.start(30)  # ~33 FPS
        else:
            self.status_label.setText("Durum: Kamera yok")

    def stop_detection(self):
        """Hedef tespit durdur"""
        self.timer.stop()
        self.status_label.setText("Durum: Durduruldu")

    def manual_guided_mode(self):
        """Manuel GUIDED mod"""
        if self.mavlink_manager.switch_to_guided():
            self.status_label.setText("Durum: GUIDED moda geçildi")
        else:
            self.status_label.setText("Durum: GUIDED mod hatası")

    def manual_auto_mode(self):
        """Manuel AUTO mod"""
        if self.mavlink_manager.switch_to_auto():
            self.status_label.setText("Durum: AUTO moda geçildi")
        else:
            self.status_label.setText("Durum: AUTO mod hatası")

    def reset_system(self):
        """Sistemi sıfırla"""
        self.mavlink_manager.reset_target_status()
        self.targets_detected_count = 0
        self.detection_status.setText("Hedef Durumu: Sıfırlandı")
        self.detection_status.setStyleSheet("color: orange; padding: 5px;")
        self.status_label.setText("Durum: Sistem sıfırlandı")

    def update_frames(self):
        """Frame güncelleme ve tespit"""
        global prev_mask_blue, prev_mask_red
        
        if not self.cam:
            return
            
        ret, frame = self.cam.read()
        if not ret:
            return

        ensure_focal_from_fov(frame.shape[1])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ref_x, ref_y = frame.shape[1]//2, frame.shape[0]//2

        disp_main = frame.copy()
        cv2.circle(disp_main,(ref_x,ref_y),6,(0,255,255),-1)

        # HSV maskeleri
        mask_blue = cv2.inRange(hsv,(90,80,50),(130,255,255))
        mask_red = cv2.inRange(hsv,(0,180,150),(10,255,255)) | cv2.inRange(hsv,(160,100,100),(179,255,255))

        # Morfolojik işlemler
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel,1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel,2)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel,1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel,2)

        # Temporal smoothing
        if prev_mask_blue is not None:
            mask_blue = cv2.addWeighted(mask_blue,alpha_mask,prev_mask_blue,1-alpha_mask,0).astype(np.uint8)
        prev_mask_blue = mask_blue.copy()
        if prev_mask_red is not None:
            mask_red = cv2.addWeighted(mask_red, alpha_mask,prev_mask_red, 1-alpha_mask,0).astype(np.uint8)
        prev_mask_red = mask_red.copy()

        # Hedef tespit
        detect_targets(mask_blue, MIN_AREA, tracked_blue)
        detect_targets(mask_red, 1000, tracked_red)

        # Görüntü hazırlama
        disp_blue = cv2.cvtColor(mask_blue,cv2.COLOR_GRAY2BGR)
        disp_red = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)

        detection_json_to_show = None
        self.debug = {}
        targets_found = False

        # Tespit edilen hedefleri işle
        for color_name, tracked_list, disp, color in [("blue", tracked_blue, disp_blue, (255,0,0)),
                                                      ("red", tracked_red, disp_red, (0,0,255))]:
            cv2.circle(disp,(ref_x,ref_y),6,(0,255,255),-1)
            for t in tracked_list:
                if not t["locked"] and t["visible_frames"]>=lock_threshold:
                    t["locked"]=True
                    
                if t["locked"]:
                    targets_found = True
                    cv2.drawContours(disp,[t["contour"]],-1,(0,0,255),2)
                    cv2.circle(disp,(t["cx"],t["cy"]),6,(0,0,255),-1)
                    
                    # Ana ekranda da göster
                    cv2.drawContours(disp_main,[t["contour"]],-1,color,3)
                    cv2.circle(disp_main,(t["cx"],t["cy"]),10,color,-1)
                    cv2.putText(disp_main, f"{color_name.upper()} TARGET", 
                               (t["cx"]+15, t["cy"]-15), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.7, color, 2, cv2.LINE_AA)

                    # JSON info hazırla
                    info = {
                        "detected": True,
                        "type": f"{color_name}_target",
                        "pixel": [t["cx"], t["cy"]],
                        "confidence": min(t["visible_frames"] / 10.0, 1.0),
                        "frames_tracked": t["visible_frames"]
                    }
                    detection_json_to_show = info

                    print(json.dumps(info, ensure_ascii=False))
                else:
                    cv2.circle(disp,(t["cx"],t["cy"]),4,color,-1)

        # Hedef tespit durumu güncelle
        if targets_found:
            self.targets_detected_count += 1
            self.detection_status.setText(f"Hedef Durumu: TESPİT EDİLDİ! ({self.targets_detected_count})")
            self.detection_status.setStyleSheet("color: lime; padding: 5px; font-weight: bold;")
            
            # Otomatik GUIDED mod kontrolü
            if (self.auto_guided_checkbox.isChecked() and 
                self.mavlink_manager.connected and 
                not self.mavlink_manager.target_detected):
                
                self.mavlink_manager.on_target_detected()
                self.status_label.setText("Durum: Hedef tespit! GUIDED moda geçiliyor...")
        else:
            if self.targets_detected_count > 0:
                self.detection_status.setText("Hedef Durumu: Kayıp")
                self.detection_status.setStyleSheet("color: orange; padding: 5px;")

        # Debug overlay
        self.draw_debug_overlay(disp_main)

        # Görüntüleri güncelle
        self.set_frame(self.label_main, disp_main)
        self.set_frame(self.label_blue, disp_blue)
        self.set_frame(self.label_red, disp_red)

        # JSON güncelle
        if detection_json_to_show is not None:
            self.last_detection_json = detection_json_to_show
            self.json_label.setText(json.dumps(detection_json_to_show, ensure_ascii=False, indent=2))

    def draw_debug_overlay(self, img):
        """Debug bilgilerini görüntüye ekle"""
        y = 24
        def put(line, color=(0,255,255)):
            nonlocal y
            cv2.putText(img, line, (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            y += 22

        put(f"HFOV: {HFOV_DEG}°")
        put(f"FOCAL_PX: {FOCAL_PX:.2f}" if FOCAL_PX else "FOCAL_PX: Hesaplanıyor...")
        put(f"TARGET_SIZE: {REAL_SIZE_M*100:.1f} cm")
        
        # MAVLink durumu
        if self.mavlink_manager.connected:
            put("MAVLink: CONNECTED", (0,255,0))
            mode_id, mode_name = self.mavlink_manager.get_current_mode()
            put(f"Mode: {mode_name}", (0,255,0) if mode_name=="GUIDED" else (255,255,0))
        else:
            put("MAVLink: DISCONNECTED", (0,0,255))
            
        # Tespit sayısı
        if self.targets_detected_count > 0:
            put(f"Targets Detected: {self.targets_detected_count}", (0,255,0))

    def set_frame(self, label, frame):
        """Frame'i Qt label'a set et"""
        rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        h,w,ch = rgb.shape
        qimg = QImage(rgb.data,w,h,ch*w,QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(label.width(),label.height(),Qt.KeepAspectRatio)
        label.setPixmap(pix)

    def closeEvent(self, event):
        """Temiz kapatma"""
        try:
            self.timer.stop()
            self.mavlink_timer.stop()
        except Exception:
            pass
            
        try:
            if self.cam:
                self.cam.stop()
        except Exception:
            pass
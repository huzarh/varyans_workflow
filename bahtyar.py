#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
İHA Hedef Tespit + MAVLink GUIDED Mod Entegre Sistem
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
from PySide6.QtCore import Qt, QTimer

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

# Global değişkenler
kernel = np.ones((5, 5), np.uint8)
MIN_AREA = 300
alpha_mask = 0.7
prev_mask_blue, prev_mask_red = None, None
tracked_blue, tracked_red = [], []
lock_threshold = 3
lost_frame_threshold = 10
position_tolerance = 60

def is_square(c, min_area=300, angle_tol=0.25, aspect_tol=0.30):
    """Konturu kare olup olmadığını kontrol et"""
    area = cv2.contourArea(c)
    if area < min_area:
        return False
    
    epsilon = 0.02 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, epsilon, True)
    
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return False
    
    x, y, w, h = cv2.boundingRect(approx)
    if h == 0:
        return False
    
    aspect_ratio = abs((w / float(h)) - 1.0)
    return aspect_ratio < aspect_tol

def detect_targets(mask, min_area, tracked_list):
    """Hedef tespit ve takip"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current_detections = []
    
    for contour in contours:
        if cv2.contourArea(contour) >= min_area and is_square(contour):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                current_detections.append({
                    "cx": cx, "cy": cy, "contour": contour,
                    "locked": False, "visible_frames": 1, "lost_frames": 0
                })
    
    # Mevcut takiplerle eşleştir
    for detection in current_detections:
        matched = False
        for track in tracked_list:
            if (abs(detection["cx"] - track["cx"]) < position_tolerance and 
                abs(detection["cy"] - track["cy"]) < position_tolerance):
                
                track.update({
                    "cx": detection["cx"], "cy": detection["cy"],
                    "contour": detection["contour"]
                })
                track["visible_frames"] += 1
                track["lost_frames"] = 0
                matched = True
                break
        
        if not matched:
            tracked_list.append(detection)
    
    # Kayıp takipleri güncelle
    for track in tracked_list:
        if not any(abs(track["cx"] - d["cx"]) < position_tolerance and 
                  abs(track["cy"] - d["cy"]) < position_tolerance 
                  for d in current_detections):
            track["lost_frames"] += 1
    
    # Eski takipleri kaldır
    tracked_list[:] = [t for t in tracked_list if t["lost_frames"] <= lost_frame_threshold]

# Kamera sınıfı
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
            bgr = self.picam.capture_array()
            if self.flip_h:
                bgr = np.ascontiguousarray(np.fliplr(bgr))
            return True, bgr
        except:
            return False, None

    def stop(self):
        try:
            self.picam.stop()
        except:
            pass

# ================== Ana Arayüz ==================
class IntegratedIHAInterface(QWidget):
    """Entegre İHA Arayüzü"""
    
    def __init__(self):
        super().__init__()
        
        self.mavlink_manager = MAVLinkManager()
        self.setup_ui()
        self.setup_camera()
        
        self.targets_detected_count = 0
        
    def setup_ui(self):
        """Arayüz kurulumu"""
        self.setWindowTitle("İHA Hedef Tespit + MAVLink Entegre Sistem")
        self.setGeometry(100, 100, 1600, 1000)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        # Başlık
        title = QLabel("İHA HEDEF TESPİT + MAVLink ENTEGRE SİSTEM")
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

        # Durum bilgileri
        self.mavlink_status = QLabel("MAVLink: Bağlı Değil")
        self.mavlink_status.setFont(QFont("Arial", 12, QFont.Bold))
        self.mavlink_status.setStyleSheet("color: red; padding: 5px;")
        
        self.current_mode = QLabel("Mod: Bilinmiyor")
        self.current_mode.setFont(QFont("Arial", 12))
        
        self.detection_status = QLabel("Hedef Durumu: Tespit Edilmedi")
        self.detection_status.setFont(QFont("Arial", 12, QFont.Bold))
        self.detection_status.setStyleSheet("color: orange; padding: 5px;")

        # Otomatik GUIDED mod seçeneği
        self.auto_guided_checkbox = QCheckBox("Hedef tespit edildiğinde otomatik GUIDED moda geç")
        self.auto_guided_checkbox.setChecked(True)
        self.auto_guided_checkbox.setStyleSheet("color: white; font-size: 12px;")

        self.status_label = QLabel("Durum: Hazır")
        self.status_label.setFont(QFont("Arial", 12))

        # Kontrol butonları
        self.btn_connect_mavlink = QPushButton("MAVLink Bağlan")
        self.btn_start = QPushButton("Tespit Başlat")
        self.btn_stop = QPushButton("Durdur")
        self.btn_guided = QPushButton("GUIDED Mod")
        self.btn_auto = QPushButton("AUTO Mod")
        self.btn_reset = QPushButton("Sıfırla")
        self.btn_exit = QPushButton("Çıkış")

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
            """)

        # Layout
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

        # Timer'lar
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)
        
        self.mavlink_timer = QTimer()
        self.mavlink_timer.timeout.connect(self.update_mavlink_status)
        self.mavlink_timer.start(1000)

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
            self.mavlink_status.setText("MAVLink: Bağlı")
            self.mavlink_status.setStyleSheet("color: green; padding: 5px;")
            self.btn_connect_mavlink.setText("Bağlı")
            self.btn_connect_mavlink.setEnabled(False)
        else:
            self.mavlink_status.setText("MAVLink: Bağlantı Hatası")
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
            self.timer.start(33)  # ~30 FPS
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

    def manual_auto_mode(self):
        """Manuel AUTO mod"""
        if self.mavlink_manager.switch_to_auto():
            self.status_label.setText("Durum: AUTO moda geçildi")

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

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        ref_x, ref_y = w//2, h//2

        # Ana görüntü kopyası
        display_main = frame.copy()
        cv2.circle(display_main, (ref_x, ref_y), 8, (0, 255, 255), -1)

        # HSV maskeleri
        mask_blue = cv2.inRange(hsv, (90, 80, 50), (130, 255, 255))
        mask_red = cv2.inRange(hsv, (0, 180, 150), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))

        # Morfolojik işlemler
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        # Temporal smoothing
        if prev_mask_blue is not None:
            mask_blue = cv2.addWeighted(mask_blue, alpha_mask, prev_mask_blue, 1-alpha_mask, 0).astype(np.uint8)
        prev_mask_blue = mask_blue.copy()
        
        if prev_mask_red is not None:
            mask_red = cv2.addWeighted(mask_red, alpha_mask, prev_mask_red, 1-alpha_mask, 0).astype(np.uint8)
        prev_mask_red = mask_red.copy()

        # Hedef tespit
        detect_targets(mask_blue, MIN_AREA, tracked_blue)
        detect_targets(mask_red, 800, tracked_red)

        # Görüntü hazırlama
        display_blue = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
        display_red = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)

        targets_found = False

        # Tespit edilen hedefleri işle
        for color_name, tracked_list, display_mask, color in [
            ("blue", tracked_blue, display_blue, (255, 0, 0)),
            ("red", tracked_red, display_red, (0, 0, 255))
        ]:
            cv2.circle(display_mask, (ref_x, ref_y), 8, (0, 255, 255), -1)
            
            for track in tracked_list:
                if not track["locked"] and track["visible_frames"] >= lock_threshold:
                    track["locked"] = True
                    
                if track["locked"]:
                    targets_found = True
                    
                    # Maskelerde göster
                    cv2.drawContours(display_mask, [track["contour"]], -1, (0, 0, 255), 2)
                    cv2.circle(display_mask, (track["cx"], track["cy"]), 8, (0, 0, 255), -1)
                    
                    # Ana ekranda göster
                    cv2.drawContours(display_main, [track["contour"]], -1, color, 3)
                    cv2.circle(display_main, (track["cx"], track["cy"]), 12, color, -1)
                    cv2.putText(display_main, f"{color_name.upper()}", 
                               (track["cx"] + 20, track["cy"] - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                else:
                    cv2.circle(display_mask, (track["cx"], track["cy"]), 6, color, -1)

        # Hedef tespit durumu güncelle
        if targets_found:
            self.targets_detected_count += 1
            self.detection_status.setText(f"Hedef Tespit Edildi! ({self.targets_detected_count})")
            self.detection_status.setStyleSheet("color: lime; padding: 5px; font-weight: bold;")
            
            # Otomatik GUIDED mod
            if (self.auto_guided_checkbox.isChecked() and 
                self.mavlink_manager.connected and 
                not self.mavlink_manager.target_detected):
                
                self.mavlink_manager.on_target_detected()
                self.status_label.setText("Durum: Hedef tespit! GUIDED moda geçiliyor...")

        # Debug bilgileri
        cv2.putText(display_main, f"Targets: {self.targets_detected_count}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        if self.mavlink_manager.connected:
            cv2.putText(display_main, "MAVLink: CONNECTED", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            mode_id, mode_name = self.mavlink_manager.get_current_mode()
            cv2.putText(display_main, f"Mode: {mode_name}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Görüntüleri güncelle
        self.set_frame(self.label_main, display_main)
        self.set_frame(self.label_blue, display_blue)
        self.set_frame(self.label_red, display_red)

    def set_frame(self, label, frame):
        """Frame'i Qt label'a set et"""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_image = QImage(rgb.data, w, h, ch * w, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        """Temiz kapatma"""
        try:
            self.timer.stop()
            self.mavlink_timer.stop()
        except:
            pass
            
        try:
            if self.cam:
                self.cam.stop()
        except:
            pass
        
        self.mavlink_manager.disconnect()
        event.accept()

# ================== Ana Çalıştırma ==================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    app.setApplicationName("İHA Hedef Tespit + MAVLink Sistem")
    app.setOrganizationName("Varyans")
    
    win = IntegratedIHAInterface()
    win.show()
    
    print("İHA Entegre Sistem başlatıldı!")
    print("Kullanım:")
    print("1. MAVLink Bağlan butonuna tıklayın")
    print("2. Tespit Başlat ile hedef arama başlatın")  
    print("3. Hedef tespit edildiğinde otomatik GUIDED moda geçer")
    
    sys.exit(app.exec())

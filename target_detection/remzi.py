#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optimized Target Detection System for Raspberry Pi 5
Camera Module 3 (IMX708) + Picamera2 + PySide6
Streamlined for performance and essential functionality only
"""

import sys
import cv2
import numpy as np
import math
import json
from PySide6.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtCore import Qt, QTimer
from picamera2 import Picamera2

# ================== CONFIGURATION ==================
# Camera Settings
CAMERA_SIZE = (1280, 720)
FLIP_HORIZONTAL = True

# Detection Parameters
MIN_AREA = 500
POSITION_TOLERANCE = 50
LOCK_THRESHOLD = 3
LOST_FRAME_THRESHOLD = 8

# Distance Calculation
REAL_SIZE_M = 0.05  # 5cm square target
HFOV_DEG = 66.0     # Standard lens FOV
USE_ALTITUDE = True  # Use altitude for distance when available

# HSV Color Ranges (adjust as needed in field)
BLUE_HSV = [(90, 80, 50), (130, 255, 255)]
RED_HSV1 = [(0, 180, 150), (10, 255, 255)]
RED_HSV2 = [(160, 100, 100), (179, 255, 255)]

# Morphological kernel for noise reduction
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# ================== CORE DETECTION FUNCTIONS ==================

class TargetDetector:
    """Handles all target detection and tracking logic"""
    
    def __init__(self):
        self.focal_px = None
        self.tracked_blue = []
        self.tracked_red = []
        self.prev_mask_blue = None
        self.prev_mask_red = None
        
    def calculate_focal_length(self, frame_width):
        """Calculate focal length from horizontal FOV"""
        if self.focal_px is None and frame_width > 0:
            self.focal_px = (0.5 * frame_width) / math.tan(math.radians(HFOV_DEG * 0.5))
    
    def is_square_contour(self, contour, min_area=MIN_AREA):
        """Check if contour resembles a square"""
        area = cv2.contourArea(contour)
        if area < min_area:
            return False
            
        # Approximate to polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            return False
            
        # Check aspect ratio
        x, y, w, h = cv2.boundingRect(approx)
        if h == 0:
            return False
        aspect_ratio = abs((w / float(h)) - 1.0)
        
        return aspect_ratio < 0.25  # Allow some tolerance for perspective
    
    def get_square_corners(self, contour):
        """Extract square corners from contour"""
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        if len(approx) != 4:
            return None
            
        pts = approx.reshape(-1, 2).astype(np.float32)
        # Sort points by angle from center
        center = np.mean(pts, axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        return pts[np.argsort(angles)]
    
    def calculate_distance(self, contour, frame_shape, altitude_m=None):
        """Calculate distance to target"""
        h, w = frame_shape[:2]
        self.calculate_focal_length(w)
        
        # Try altitude-based calculation first
        if USE_ALTITUDE and altitude_m is not None:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                if self.focal_px:
                    u, v = float(cx), float(cy)
                    cx_cam, cy_cam = w/2.0, h/2.0
                    x = (u - cx_cam) / self.focal_px
                    y = (v - cy_cam) / self.focal_px
                    
                    slant_distance = altitude_m * math.sqrt(x*x + y*y + 1.0)
                    return slant_distance
        
        # Fallback to size-based calculation
        corners = self.get_square_corners(contour)
        if corners is None or self.focal_px is None:
            return None
            
        # Calculate average side length in pixels
        distances = []
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i + 1) % 4]
            distances.append(np.linalg.norm(p1 - p2))
        
        avg_side_px = np.mean(distances)
        if avg_side_px > 1:
            return (self.focal_px * REAL_SIZE_M) / avg_side_px
            
        return None
    
    def detect_and_track(self, mask, tracked_list, min_area=MIN_AREA):
        """Detect targets in mask and update tracking"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        current_detections = []
        for contour in contours:
            if self.is_square_contour(contour, min_area):
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    current_detections.append({
                        "cx": cx, "cy": cy, "contour": contour,
                        "locked": False, "visible_frames": 1, "lost_frames": 0
                    })
        
        # Match with existing tracks
        for detection in current_detections:
            matched = False
            for track in tracked_list:
                if (abs(detection["cx"] - track["cx"]) < POSITION_TOLERANCE and 
                    abs(detection["cy"] - track["cy"]) < POSITION_TOLERANCE):
                    
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
        
        # Update lost tracks
        for track in tracked_list:
            if not any(abs(track["cx"] - d["cx"]) < POSITION_TOLERANCE and 
                      abs(track["cy"] - d["cy"]) < POSITION_TOLERANCE 
                      for d in current_detections):
                track["lost_frames"] += 1
        
        # Remove old tracks
        tracked_list[:] = [t for t in tracked_list if t["lost_frames"] <= LOST_FRAME_THRESHOLD]
        
        # Lock stable tracks
        for track in tracked_list:
            if not track["locked"] and track["visible_frames"] >= LOCK_THRESHOLD:
                track["locked"] = True
    
    def process_frame(self, frame, altitude_m=None):
        """Main frame processing pipeline"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        
        # Create masks
        mask_blue = cv2.inRange(hsv, BLUE_HSV[0], BLUE_HSV[1])
        mask_red = (cv2.inRange(hsv, RED_HSV1[0], RED_HSV1[1]) | 
                   cv2.inRange(hsv, RED_HSV2[0], RED_HSV2[1]))
        
        # Morphological operations for noise reduction
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, MORPH_KERNEL)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # Temporal smoothing
        alpha = 0.7
        if self.prev_mask_blue is not None:
            mask_blue = cv2.addWeighted(mask_blue, alpha, self.prev_mask_blue, 1-alpha, 0)
        if self.prev_mask_red is not None:
            mask_red = cv2.addWeighted(mask_red, alpha, self.prev_mask_red, 1-alpha, 0)
        
        self.prev_mask_blue = mask_blue.copy()
        self.prev_mask_red = mask_red.copy()
        
        # Detect and track
        self.detect_and_track(mask_blue, self.tracked_blue)
        self.detect_and_track(mask_red, self.tracked_red, min_area=800)
        
        # Generate results
        results = []
        for color, tracked_list in [("blue", self.tracked_blue), ("red", self.tracked_red)]:
            for track in tracked_list:
                if track["locked"]:
                    distance = self.calculate_distance(track["contour"], frame.shape, altitude_m)
                    
                    result = {
                        "detected": True,
                        "type": f"{color}_target",
                        "pixel": [track["cx"], track["cy"]],
                        "distance_m": round(distance, 2) if distance else None,
                        "confidence": min(track["visible_frames"] / 10.0, 1.0)
                    }
                    results.append(result)
        
        return results, mask_blue, mask_red

# ================== CAMERA WRAPPER ==================

class OptimizedPiCam:
    """Optimized PiCamera2 wrapper for RPi5"""
    
    def __init__(self, size=CAMERA_SIZE, flip_h=FLIP_HORIZONTAL):
        self.picam = Picamera2()
        self.flip_h = flip_h
        
        # Optimized configuration for performance
        config = self.picam.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            buffer_count=2  # Reduce buffer count for lower latency
        )
        self.picam.configure(config)
        self.picam.start()
    
    def read(self):
        """Capture and return BGR frame"""
        try:
            rgb = self.picam.capture_array()
            if self.flip_h:
                rgb = np.ascontiguousarray(np.fliplr(rgb))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return True, bgr
        except Exception as e:
            print(f"Camera error: {e}")
            return False, None
    
    def stop(self):
        """Clean shutdown"""
        try:
            self.picam.stop()
        except Exception:
            pass

# ================== MINIMAL GUI ==================

class TargetDetectionGUI(QWidget):
    """Streamlined GUI for target detection"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.setup_camera()
        self.detector = TargetDetector()
        
    def setup_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("🎯 Target Detection System")
        self.setGeometry(100, 100, 1400, 800)
        self.setStyleSheet("background-color: #2b2b2b; color: white;")
        
        # Title
        title = QLabel("🎯 TARGET DETECTION SYSTEM")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #00ff00; margin: 10px;")
        
        # Video display
        self.video_label = QLabel("Camera Feed")
        self.video_label.setFixedSize(1280, 720)
        self.video_label.setStyleSheet("border: 2px solid #555; background: black;")
        self.video_label.setAlignment(Qt.AlignCenter)
        
        # Status and info
        self.status_label = QLabel("Status: Ready")
        self.status_label.setFont(QFont("Arial", 12))
        
        self.info_label = QLabel("No targets detected")
        self.info_label.setFont(QFont("Consolas", 10))
        self.info_label.setStyleSheet("background: #1a1a1a; border: 1px solid #444; padding: 10px;")
        
        # Control buttons
        self.btn_start = QPushButton("▶ Start Detection")
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_exit = QPushButton("❌ Exit")
        
        for btn in [self.btn_start, self.btn_stop, self.btn_exit]:
            btn.setFixedHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    background: #0066cc; color: white; font-size: 14px;
                    border-radius: 6px; font-weight: bold;
                }
                QPushButton:hover { background: #0080ff; }
                QPushButton:pressed { background: #004499; }
            """)
        
        # Layout
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.btn_start)
        button_layout.addWidget(self.btn_stop)
        button_layout.addWidget(self.btn_exit)
        
        info_layout = QHBoxLayout()
        info_layout.addWidget(self.status_label)
        info_layout.addStretch()
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(title)
        main_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        main_layout.addLayout(info_layout)
        main_layout.addWidget(self.info_label)
        main_layout.addLayout(button_layout)
        
        self.setLayout(main_layout)
        
        # Connect signals
        self.btn_start.clicked.connect(self.start_detection)
        self.btn_stop.clicked.connect(self.stop_detection)
        self.btn_exit.clicked.connect(self.close)
        
        # Timer for frame updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        
    def setup_camera(self):
        """Initialize camera"""
        try:
            self.camera = OptimizedPiCam()
            self.status_label.setText("Status: Camera ready")
        except Exception as e:
            self.status_label.setText(f"Status: Camera error - {e}")
            self.camera = None
    
    def start_detection(self):
        """Start target detection"""
        if self.camera:
            self.status_label.setText("Status: Detection active")
            self.timer.start(33)  # ~30 FPS
        else:
            self.status_label.setText("Status: No camera available")
    
    def stop_detection(self):
        """Stop target detection"""
        self.timer.stop()
        self.status_label.setText("Status: Detection stopped")
    
    def update_frame(self):
        """Process and display frame"""
        if not self.camera:
            return
            
        ret, frame = self.camera.read()
        if not ret:
            self.status_label.setText("Status: Camera read error")
            return
        
        # Get mock altitude (replace with real sensor data)
        altitude = self.get_altitude()
        
        # Process frame
        results, mask_blue, mask_red = self.detector.process_frame(frame, altitude)
        
        # Draw results on frame
        display_frame = frame.copy()
        self.draw_results(display_frame, results)
        
        # Update display
        self.display_frame(display_frame)
        self.update_info(results)
    
    def get_altitude(self):
        """Mock altitude function - replace with real sensor"""
        return None  # Return actual altitude in meters
    
    def draw_results(self, frame, results):
        """Draw detection results on frame"""
        h, w = frame.shape[:2]
        
        # Draw crosshair
        cv2.line(frame, (w//2-20, h//2), (w//2+20, h//2), (0, 255, 255), 2)
        cv2.line(frame, (w//2, h//2-20), (w//2, h//2+20), (0, 255, 255), 2)
        
        # Draw detections
        for result in results:
            x, y = result["pixel"]
            color = (255, 0, 0) if "blue" in result["type"] else (0, 0, 255)
            
            # Draw target marker
            cv2.circle(frame, (x, y), 10, color, 3)
            cv2.circle(frame, (x, y), 20, color, 2)
            
            # Draw info text
            text = f"{result['type']}"
            if result["distance_m"]:
                text += f" {result['distance_m']}m"
            
            cv2.putText(frame, text, (x + 25, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Draw debug info
        y_pos = 30
        debug_info = [
            f"HFOV: {HFOV_DEG}°",
            f"Focal: {self.detector.focal_px:.1f}px" if self.detector.focal_px else "Focal: calculating...",
            f"Targets: {len(results)}"
        ]
        
        for info in debug_info:
            cv2.putText(frame, info, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y_pos += 25
    
    def display_frame(self, frame):
        """Convert and display frame in Qt"""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        
        qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        
        # Scale to fit label
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(), 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        
        self.video_label.setPixmap(scaled_pixmap)
    
    def update_info(self, results):
        """Update information display"""
        if not results:
            self.info_label.setText("No targets detected")
            return
            
        info_lines = []
        for i, result in enumerate(results):
            line = f"Target {i+1}: {result['type']} at {result['pixel']}"
            if result["distance_m"]:
                line += f" | Distance: {result['distance_m']}m"
            line += f" | Confidence: {result['confidence']:.2f}"
            info_lines.append(line)
        
        self.info_label.setText("\n".join(info_lines))
    
    def closeEvent(self, event):
        """Clean shutdown"""
        self.timer.stop()
        if self.camera:
            self.camera.stop()
        event.accept()

# ================== MAIN EXECUTION ==================

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Set application properties
    app.setApplicationName("Target Detection System")
    app.setOrganizationName("Varyans")
    
    # Create and show main window
    window = TargetDetectionGUI()
    window.show()
    
    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
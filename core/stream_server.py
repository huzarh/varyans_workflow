#!/usr/bin/env python3
"""
Raspberry Pi 5 için HTTP Stream Server
Sadece video stream - performans odaklı
25-35 FPS, 600x300px boyutunda yayın
"""
import cv2
import numpy as np
import threading
import time
import json
from typing import Optional, Dict, Any
from http.server import HTTPServer, BaseHTTPRequestHandler
import socketserver
from io import BytesIO
import base64

class StreamHandler(BaseHTTPRequestHandler):
    """HTTP Stream Handler - Sadece video"""
    
    def do_GET(self):
        """GET isteklerini işle"""
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(self.get_simple_html().encode())
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            self.stream_video()
        else:
            self.send_response(404)
            self.end_headers()
    
    def get_simple_html(self) -> str:
        """Sade video HTML sayfası"""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>İHA Stream</title>
    <style>
        body { 
            margin: 0; 
            padding: 0; 
            background: #000; 
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }
        img { 
            max-width: 100vw;
            max-height: 100vh;
            object-fit: contain;
        }
    </style>
</head>
<body>
    <img src="/stream" alt="İHA Stream">
</body>
</html>
        """
    
    def stream_video(self):
        """Video stream gönder - 25-35 FPS"""
        frame_time = 1.0 / 30.0  # 30 FPS (25-35 arası ortalama)
        
        while True:
            if hasattr(self.server, 'current_frame') and self.server.current_frame is not None:
                # Frame'i 600x300 boyutuna küçült
                resized_frame = cv2.resize(self.server.current_frame, (600, 300))
                
                # Frame'i JPEG'e çevir (optimize edilmiş kalite)
                _, buffer = cv2.imencode('.jpg', resized_frame, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()
                
                # HTTP multipart response
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame_bytes)))
                self.end_headers()
                self.wfile.write(frame_bytes)
                self.wfile.write(b'\r\n')
                
                # FPS kontrolü için bekleme
                time.sleep(frame_time)
            else:
                time.sleep(frame_time)

class StreamServer:
    """HTTP Stream Server - Sadece video, performans odaklı"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8080):
        self.host = host
        self.port = port
        self.current_frame: Optional[np.ndarray] = None
        self.server = None
        self.thread = None
        self.running = False
        self.target_width = 600
        self.target_height = 300
    
    def start(self):
        """Server'ı başlat"""
        try:
            self.server = HTTPServer((self.host, self.port), StreamHandler)
            self.server.current_frame = self.current_frame
            
            self.thread = threading.Thread(target=self.server.serve_forever)
            self.thread.daemon = True
            self.thread.start()
            
            self.running = True
            print(f"🌐 Stream Server başlatıldı: http://{self.host}:{self.port}")
            print(f"📱 Mobil erişim: http://172.20.10.4:{self.port}")
            print(f"📐 Stream boyutu: {self.target_width}x{self.target_height}px")
            print(f"🎬 FPS: 25-35 arası (hedef: 30 FPS)")
            print(f"⚡ Performans modu: Sadece video stream")
            
        except Exception as e:
            print(f"❌ Stream Server başlatılamadı: {e}")
    
    def stop(self):
        """Server'ı durdur"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        self.running = False
        print("🛑 Stream Server durduruldu")
    
    def update_frame(self, frame: np.ndarray):
        """Frame'i güncelle - orijinal boyutta sakla, stream'de küçült"""
        self.current_frame = frame.copy()
        if self.server:
            self.server.current_frame = self.current_frame

# Global stream server instance
stream_server = StreamServer()

if __name__ == "__main__":
    # Test için
    server = StreamServer()
    server.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()

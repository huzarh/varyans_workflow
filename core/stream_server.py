#!/usr/bin/env python3
"""
Raspberry Pi 5 için HTTP Stream Server
Görüntü işleme sonuçlarını web üzerinden gözlemleme
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
    """HTTP Stream Handler"""
    
    def do_GET(self):
        """GET isteklerini işle"""
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(self.get_html_page().encode())
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            self.stream_video()
        elif self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(self.server.detection_data).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def get_html_page(self) -> str:
        """Ana HTML sayfasını oluştur"""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>İHA Target Detection Stream</title>
    <style>
        body { 
            margin: 0; 
            padding: 20px; 
            background: #1e1e1e; 
            color: white; 
            font-family: Arial, sans-serif;
        }
        .container { 
            display: flex; 
            gap: 20px; 
            flex-wrap: wrap;
        }
        .video-container { 
            position: relative; 
            border: 2px solid #444; 
            border-radius: 8px;
            overflow: hidden;
        }
        .video-container h3 { 
            position: absolute; 
            top: 10px; 
            left: 10px; 
            background: rgba(0,0,0,0.7); 
            padding: 5px 10px; 
            border-radius: 4px;
            margin: 0;
        }
        img { 
            display: block; 
            max-width: 100%; 
            height: auto;
        }
        .data-panel {
            background: #2a2a2a;
            padding: 15px;
            border-radius: 8px;
            min-width: 300px;
            max-height: 400px;
            overflow-y: auto;
        }
        .status {
            background: #333;
            padding: 10px;
            border-radius: 4px;
            margin-bottom: 10px;
        }
        .detection-info {
            background: #444;
            padding: 10px;
            border-radius: 4px;
            margin-bottom: 10px;
        }
        .blue { color: #4fc3f7; }
        .red { color: #f44336; }
        .green { color: #4caf50; }
    </style>
</head>
<body>
    <h1>🚁 İHA Target Detection Stream</h1>
    <div class="container">
        <div class="video-container">
            <h3>Ana Görsel</h3>
            <img src="/stream" alt="Main Stream">
        </div>
        <div class="data-panel">
            <h3>📊 Detection Data</h3>
            <div class="status">
                <div><strong>Status:</strong> <span id="status">Connecting...</span></div>
                <div><strong>FPS:</strong> <span id="fps">0</span></div>
                <div><strong>Frame Count:</strong> <span id="frameCount">0</span></div>
            </div>
            <div class="detection-info">
                <h4>🔵 Blue Targets</h4>
                <div id="blueTargets">None detected</div>
            </div>
            <div class="detection-info">
                <h4>🔴 Red Targets</h4>
                <div id="redTargets">None detected</div>
            </div>
        </div>
    </div>

    <script>
        let frameCount = 0;
        let lastTime = Date.now();
        
        // FPS hesaplama
        function updateFPS() {
            const now = Date.now();
            const fps = Math.round(1000 / (now - lastTime));
            document.getElementById('fps').textContent = fps;
            lastTime = now;
        }
        
        // Stream güncelleme
        const img = document.querySelector('img');
        img.onload = function() {
            frameCount++;
            document.getElementById('frameCount').textContent = frameCount;
            updateFPS();
        };
        
        // Detection data güncelleme
        function updateDetectionData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').textContent = 'Connected';
                    
                    // Blue targets
                    const blueDiv = document.getElementById('blueTargets');
                    if (data.blue && data.blue.detected) {
                        blueDiv.innerHTML = `
                            <div class="blue">
                                <div>Type: ${data.blue.type}</div>
                                <div>Pixel: [${data.blue.pixel[0]}, ${data.blue.pixel[1]}]</div>
                                <div>Distance: ${data.blue.distance_m || 'N/A'}m</div>
                                <div>Confidence: ${data.blue.confidence}</div>
                                <div>Area: ${data.blue.bbox_area}px²</div>
                            </div>
                        `;
                    } else {
                        blueDiv.textContent = 'None detected';
                    }
                    
                    // Red targets
                    const redDiv = document.getElementById('redTargets');
                    if (data.red && data.red.detected) {
                        redDiv.innerHTML = `
                            <div class="red">
                                <div>Type: ${data.red.type}</div>
                                <div>Pixel: [${data.red.pixel[0]}, ${data.red.pixel[1]}]</div>
                                <div>Distance: ${data.red.distance_m || 'N/A'}m</div>
                                <div>Confidence: ${data.red.confidence}</div>
                                <div>Area: ${data.red.bbox_area}px²</div>
                            </div>
                        `;
                    } else {
                        redDiv.textContent = 'None detected';
                    }
                })
                .catch(error => {
                    document.getElementById('status').textContent = 'Error: ' + error.message;
                });
        }
        
        // Her 100ms'de data güncelle
        setInterval(updateDetectionData, 100);
        
        // İlk yükleme
        updateDetectionData();
    </script>
</body>
</html>
        """
    
    def stream_video(self):
        """Video stream gönder"""
        while True:
            if hasattr(self.server, 'current_frame') and self.server.current_frame is not None:
                # Frame'i JPEG'e çevir
                _, buffer = cv2.imencode('.jpg', self.server.current_frame, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()
                
                # HTTP multipart response
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame_bytes)))
                self.end_headers()
                self.wfile.write(frame_bytes)
                self.wfile.write(b'\r\n')
            else:
                time.sleep(0.033)  # 30 FPS

class StreamServer:
    """HTTP Stream Server"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8080):
        self.host = host
        self.port = port
        self.current_frame: Optional[np.ndarray] = None
        self.detection_data: Dict[str, Any] = {
            'blue': {'detected': False},
            'red': {'detected': False},
            'timestamp': time.time()
        }
        self.server = None
        self.thread = None
        self.running = False
    
    def start(self):
        """Server'ı başlat"""
        try:
            self.server = HTTPServer((self.host, self.port), StreamHandler)
            self.server.current_frame = self.current_frame
            self.server.detection_data = self.detection_data
            
            self.thread = threading.Thread(target=self.server.serve_forever)
            self.thread.daemon = True
            self.thread.start()
            
            self.running = True
            print(f"🌐 Stream Server başlatıldı: http://{self.host}:{self.port}")
            print(f"📱 Mobil erişim: http://[RASPBERRY_PI_IP]:{self.port}")
            
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
        """Frame'i güncelle"""
        self.current_frame = frame.copy()
        if self.server:
            self.server.current_frame = self.current_frame
    
    def update_detection_data(self, blue_data: Optional[Dict[str, Any]] = None, 
                            red_data: Optional[Dict[str, Any]] = None):
        """Detection data'yı güncelle"""
        if blue_data:
            self.detection_data['blue'] = blue_data
        if red_data:
            self.detection_data['red'] = red_data
        
        self.detection_data['timestamp'] = time.time()
        
        if self.server:
            self.server.detection_data = self.detection_data

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

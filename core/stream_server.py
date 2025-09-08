#!/usr/bin/env python3
"""
Minimal Stream Server (Raspberry Pi)
600x300 çözünürlük, 25-35 FPS
Kullanım:
    from core.stream_server import StreamServer
"""

import cv2, time, threading, socket
from http.server import HTTPServer, BaseHTTPRequestHandler

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><body><img src='/stream'></body></html>")
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                frame = getattr(self.server, "current_frame", None)
                if frame is not None:
                    resized = cv2.resize(frame, (600, 300))
                    _, buf = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                        buf.tobytes() +
                        b"\r\n"
                    )
                time.sleep(1/30)  # 30 FPS hedef (25–35 arası)

def get_ip():
    """Yerel IPv4 adresini al"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

class StreamServer:
    def __init__(self, host="0.0.0.0", port=8080, cam_index=0):
        self.host = host
        self.port = port
        self.cam_index = cam_index
        self.server = HTTPServer((self.host, self.port), Handler)
        self.server.current_frame = None
        self.running = False

    def start(self):
        """Sunucuyu başlat ve kamera loop aç"""
        def capture_loop():
            cap = cv2.VideoCapture(self.cam_index)
            while cap.isOpened():
                ret, f = cap.read()
                if ret:
                    self.server.current_frame = f

        threading.Thread(target=capture_loop, daemon=True).start()
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.running = True
        ip = get_ip()
        print(f"🌐 Stream aktif: http://{ip}:{self.port}")
        print("🎬 FPS: 25–35 arası | Boyut: 600x300")

    def stop(self):
        """Sunucuyu durdur"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        self.running = False
        print("🛑 Stream Server kapatıldı")

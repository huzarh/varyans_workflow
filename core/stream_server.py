#!/usr/bin/env python3
"""
Minimal Raspberry Pi HTTP Video Stream
600x300, 25-35 FPS
"""

import cv2, time, threading, socket
from http.server import HTTPServer, BaseHTTPRequestHandler

frame = None

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
                if frame is not None:
                    resized = cv2.resize(frame, (600, 300))
                    _, buf = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
                    )
                time.sleep(1/30)  # 30 FPS hedefi (25–35 arası)

def capture_loop():
    global frame
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, f = cap.read()
        if ret:
            frame = f

if __name__ == "__main__":
    threading.Thread(target=capture_loop, daemon=True).start()
    ip = get_ip()
    port = 8080
    print(f"🌐 Stream aktif: http://{ip}:{port}")
    HTTPServer(("0.0.0.0", port), Handler).serve_forever()

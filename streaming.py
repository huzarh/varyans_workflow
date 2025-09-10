import time
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import socket
import threading

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): return
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type","text/html; charset=utf-8")
            self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            html = ("<html><body style='margin:0;background:#111;color:#eee;'>"
                    "<div style='padding:8px;font-family:monospace'>/stream</div>"
                    "<img src='/stream' style='width:100%;height:auto;'/>"
                    "</body></html>")
            self.wfile.write(html.encode("utf-8")); return
        if self.path != "/stream":
            self.send_error(404); return
        self.send_response(200)
        self.send_header("Content-type","multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        if not hasattr(self.server, "_stream_cnt"):
            self.server._stream_cnt = 0; self.server._stream_t0 = time.time()
        try:
            last_send = 0.0; min_dt = 1.0 / self.server.target_fps
            while True:
                now = time.time()
                if now - last_send < min_dt:
                    time.sleep(min_dt - (now - last_send)); continue
                frame = getattr(self.server, "current_frame", None)
                if frame is None:
                    time.sleep(0.005); continue
                h, w = frame.shape[:2]
                target_w = min(self.server.max_width, w)
                target_h = int(h * (target_w / float(w)))
                out = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, self.server.jpeg_quality])
                if not ok:
                    time.sleep(0.005); continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(buf.tobytes()); self.wfile.write(b"\r\n")
                self.server._stream_cnt += 1; last_send = now
        except (BrokenPipeError, ConnectionResetError): pass
        except Exception: pass

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try: s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]
    except Exception: ip = "127.0.0.1"
    finally: s.close()
    return ip

def start_streaming_server(port=8080, target_fps=30, max_width=480, jpeg_quality=60):
    server = ThreadingHTTPServer(("0.0.0.0", port), MJPEGHandler)
    server.current_frame = None
    server.target_fps = target_fps
    server.max_width = max_width
    server.jpeg_quality = jpeg_quality
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    try: print(f"Stream:  http://{get_ip()}:{port}/stream")
    except Exception: print(f"http://localhost:{port}/stream")
    return server

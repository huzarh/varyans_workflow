import cv2
import time
import threading
import socket
import queue
from http.server import HTTPServer, BaseHTTPRequestHandler
from io import BytesIO

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # HTTP log mesajlarını kapat
    
    def do_GET(self):
        try:
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-type", "text/html")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                html = b"""
                <html>
                <head><title>Video Stream</title></head>
                <body style='margin:0; background:#000; display:flex; justify-content:center; align-items:center; height:100vh;'>
                    <img src='/stream' style='max-width:100%; max-height:100%;'>
                </body>
                </html>
                """
                self.wfile.write(html)
                
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.end_headers()
                
                self._stream_frames()
            else:
                self.send_response(404)
                self.end_headers()
                
        except Exception as e:
            print(f"HTTP Handler Error: {e}")
            
    def _stream_frames(self):
        """Frame stream işlemi - buffer kontrolü ile"""
        frame_queue = self.server.frame_queue
        last_frame_time = time.time()
        
        try:
            while True:
                try:
                    # Queue'dan frame al (timeout ile)
                    frame_data = frame_queue.get(timeout=0.1)
                    
                    if frame_data is None:
                        break
                        
                    # Frame gönder
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame_data)}\r\n\r\n".encode())
                    self.wfile.write(frame_data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    
                    last_frame_time = time.time()
                    
                except queue.Empty:
                    # Timeout durumunda bağlantıyı kontrol et
                    if time.time() - last_frame_time > 5.0:  # 5 saniye timeout
                        print("Stream timeout - bağlantı kesildi")
                        break
                    continue
                    
        except (ConnectionResetError, BrokenPipeError):
            print("Client bağlantısı kesildi")
        except Exception as e:
            print(f"Stream error: {e}")

def get_local_ip():
    """Yerel IP adresini güvenli şekilde al"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except:
        return "127.0.0.1"

class StableStreamServer:
    def __init__(self, host="0.0.0.0", port=8080, cam_index=0):
        self.host = host
        self.port = port
        self.cam_index = cam_index
        self.server = None
        self.running = False
        self.capture_thread = None
        self.server_thread = None
        
        # Frame queue - buffer kontrolü için
        self.frame_queue = queue.Queue(maxsize=2)  # Maksimum 2 frame buffer
        
    def _capture_loop(self):
        """Kamera capture loop - optimize edilmiş"""
        cap = cv2.VideoCapture(self.cam_index)
        
        # Kamera ayarları
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer boyutunu küçült
        
        print(f"Kamera başlatıldı: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        
        frame_time = 1.0 / 25  # 25 FPS hedef
        last_time = time.time()
        
        try:
            while self.running and cap.isOpened():
                ret, frame = cap.read()
                
                if not ret:
                    print("Frame okunamadı, kamera yeniden başlatılıyor...")
                    cap.release()
                    time.sleep(1)
                    cap = cv2.VideoCapture(self.cam_index)
                    continue
                
                # FPS kontrolü
                current_time = time.time()
                if current_time - last_time < frame_time:
                    time.sleep(frame_time - (current_time - last_time))
                
                # Frame'i yeniden boyutlandır ve encode et
                resized_frame = cv2.resize(frame, (600, 300))
                
                # JPEG encode parametreleri - hız odaklı
                encode_params = [
                    cv2.IMWRITE_JPEG_QUALITY, 75,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 1
                ]
                
                ret, buffer = cv2.imencode('.jpg', resized_frame, encode_params)
                
                if ret:
                    frame_data = buffer.tobytes()
                    
                    # Queue'ya frame ekle (eski frame'leri at)
                    try:
                        # Queue dolu ise eski frame'i at
                        if self.frame_queue.full():
                            try:
                                self.frame_queue.get_nowait()
                            except queue.Empty:
                                pass
                        
                        self.frame_queue.put_nowait(frame_data)
                        
                    except queue.Full:
                        pass  # Queue dolu, frame'i atla
                
                last_time = time.time()
                
        except Exception as e:
            print(f"Capture error: {e}")
        finally:
            cap.release()
            print("Kamera kapatıldı")
    
    def start(self, enable_capture=False):
        """Sunucuyu başlat
        enable_capture=True ise dahili kamera yakalama başlar, aksi halde yalnız itme (push) modu kullanılır.
        """
        if self.running:
            print("Server zaten çalışıyor!")
            return
            
        try:
            # HTTP Server oluştur
            self.server = HTTPServer((self.host, self.port), StreamHandler)
            self.server.frame_queue = self.frame_queue
            
            # Capture thread başlat (opsiyonel)
            self.running = True
            if enable_capture:
                self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self.capture_thread.start()
            
            # Server thread başlat
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()
            
            local_ip = get_local_ip()
            print(f"🌐 Stream Server başlatıldı!")
            print(f"📡 URL: http://{local_ip}:{self.port}")
            print(f"🎬 Hedef FPS: 25 | Boyut: 300x100")
            print(f"💾 Buffer Size: {self.frame_queue.maxsize} frame")
            
        except Exception as e:
            print(f"Server başlatma hatası: {e}")
            self.running = False
    
    def update_frame(self, frame):
        """Manuel frame güncellemesi (harici kullanım için)"""
        if not self.running:
            return
            
        try:
            # Frame'i işle
            resized_frame = cv2.resize(frame, (300, 100))
            
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 75]
            ret, buffer = cv2.imencode('.jpg', resized_frame, encode_params)
            
            if ret:
                frame_data = buffer.tobytes()
                
                # Queue'ya ekle
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                try:
                    self.frame_queue.put_nowait(frame_data)
                except queue.Full:
                    pass
                    
        except Exception as e:
            print(f"Frame update error: {e}")
    
    def stop(self):
        """Sunucuyu durdur"""
        print("🛑 Server kapatılıyor...")
        self.running = False
        
        # Queue'yu temizle
        try:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait()
        except queue.Empty:
            pass
        
        # Queue'ya None ekle (stream'leri sonlandır)
        try:
            self.frame_queue.put_nowait(None)
        except queue.Full:
            pass
        
        # Server'ı kapat
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        
        print("✅ Server kapatıldı")
    
    def is_running(self):
        return self.running

# Kullanım örneği
if __name__ == "__main__":
    server = StableStreamServer(port=8080, cam_index=0)
    
    try:
        server.start(enable_capture=True)
        print("Çıkmak için Ctrl+C basın...")
        
        # Ana loop
        while server.is_running():
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nKapatılıyor...")
    finally:
        server.stop()
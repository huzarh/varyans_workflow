#!/usr/bin/env python3
"""
Uçuş cihazında GUI olmadan kamera akışını sunan basit MJPEG server.
- Tercihen Picamera2 kullanır, değilse OpenCV (V4L2) ile devam eder.
- Kamera boyut/fps değerleri core.config içindeki camera_config'ten alınır.
- Yayın uç noktası: /ucus_arayuz
"""
import os
import platform
import sys
import time
from typing import Generator, Optional, Tuple

# GUI plugin uyarılarını engellemek için güvenli offscreen ayarı
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
from flask import Flask, Response, request
from flask_cors import CORS
import socket

# Proje köküne import yolu ekle
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from core.config import camera_config  # noqa: E402

# Picamera2 opsiyonel
PICAM_AVAILABLE = False
try:
    from picamera2 import Picamera2  # type: ignore

    PICAM_AVAILABLE = True
except Exception:
    PICAM_AVAILABLE = False


class CameraSource:
    """Picamera2 mevcutsa onu, değilse OpenCV'yi kullanan kamera soyutlaması."""

    def __init__(self, width: Optional[int] = None, height: Optional[int] = None, fps: Optional[int] = None):
        self.width = int(width or camera_config.width)
        self.height = int(height or camera_config.height)
        self.fps = int(fps or camera_config.fps)

        self.source_type = "none"
        self.picam: Optional["Picamera2"] = None
        self.cap: Optional[cv2.VideoCapture] = None

        self._open()

    def _open(self) -> None:
        if PICAM_AVAILABLE:
            try:
                self.picam = Picamera2()
                cfg = self.picam.create_video_configuration(
                    main={"size": (self.width, self.height)},
                    controls={"FrameRate": self.fps},
                )
                self.picam.configure(cfg)
                self.picam.start()
                self.source_type = "picamera2"
                return
            except Exception as e:
                print(f"Picamera2 açılamadı, V4L2'ye geçiliyor: {e}")

        backend = cv2.CAP_V4L2 if platform.system() != "Windows" else cv2.CAP_DSHOW
        self.cap = cv2.VideoCapture(0, backend)
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError("Kamera açılamadı (ne Picamera2 ne V4L2)")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.source_type = "opencv"

    def read_bgr(self) -> Tuple[bool, Optional["cv2.typing.MatLike"]]:
        try:
            if self.source_type == "picamera2" and self.picam is not None:
                rgb = self.picam.capture_array()  # RGB gelir
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                return True, bgr
            elif self.source_type == "opencv" and self.cap is not None:
                return self.cap.read()
            else:
                return False, None
        except Exception as e:
            print(f"Kare okunamadı: {e}")
            return False, None

    def release(self) -> None:
        try:
            if self.source_type == "picamera2" and self.picam is not None:
                self.picam.stop()
            if self.source_type == "opencv" and self.cap is not None:
                self.cap.release()
        except Exception:
            pass


def get_primary_ip() -> str:
    """Cihazın yerel ağdaki birincil IP adresini tespit eder."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"


app = Flask(__name__)
CORS(app, resources={r"/ucus_arayuz": {"origins": "*"}})

# Kamera kaynağını global başlat
camera: Optional[CameraSource] = None
first_client_logged = False


def init_camera() -> None:
    global camera
    if camera is None:
        camera = CameraSource()


@app.route("/ucus_arayuz")
def video_feed() -> Response:
    global first_client_logged
    init_camera()

    # İlk istemci loglama
    if not first_client_logged:
        client_ip = request.remote_addr or "unknown"
        print(f"İlk istemci bağlandı: {client_ip} -> /ucus_arayuz")
        first_client_logged = True

    def generate_frames() -> Generator[bytes, None, None]:
        assert camera is not None
        # JPEG kalite parametresi
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        while True:
            ok, frame_bgr = camera.read_bgr()
            if not ok or frame_bgr is None:
                # Aşırı döngüyü engellemek için hafif bekleme
                time.sleep(0.01)
                continue
            try:
                ret, buffer = cv2.imencode(".jpg", frame_bgr, encode_params)
                if not ret:
                    continue
                frame = buffer.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            except Exception as e:
                print(f"JPEG encode hatası: {e}")
                continue

    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def root() -> str:
    return "MJPEG stream: /ucus_arayuz"


if __name__ == "__main__":
    PORT = 5000
    try:
        init_camera()
        ip = get_primary_ip()
        print(
            f"Kamera başlatıldı: {camera.source_type if camera else 'none'} | "
            f"{camera.width}x{camera.height}@{camera.fps}"
        )
        print("Yayın adresleri:")
        print(f"- http://{ip}:{PORT}/ucus_arayuz")
        print(f"- http://127.0.0.1:{PORT}/ucus_arayuz")
        print("Tarayıcıyla açın veya MJPEG destekli istemciyle bağlanın.")
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        if camera is not None:
            camera.release() 
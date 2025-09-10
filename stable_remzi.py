#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, cv2, math, json, time, socket, threading, argparse, platform, signal, shutil, sys
import numpy as np
from streaming import start_streaming_server

os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "3")

STREAM_TARGET_FPS   = 30
STREAM_MAX_WIDTH    = 480
STREAM_JPEG_QUALITY = 60

REAL_SIZE_M = 0.05
USE_HFOV   = True
HFOV_DEG   = 66.0
FOCAL_PX   = None

BLUE_H_LO, BLUE_H_HI = 95, 130
BLUE_S_LO, BLUE_V_LO = 120, 70
WHITE_S_HI, WHITE_V_LO = 90, 200
B_DOM_MARGIN = 25

MIN_AREA = 300
lock_threshold, lost_frame_threshold, position_tolerance = 3, 10, 60

ACCEPT_DIST_MIN_M = 1.0
ACCEPT_DIST_MAX_M = 50.0
NEAR_REJECT_SIDE_PX = 180

kernel = np.ones((3,3), np.uint8)
alpha_mask = 1
prev_mask_blue, prev_mask_red = None, None
tracked_blue, tracked_red = [], []

class CamBase:
    def read(self): raise NotImplementedError
    def set_auto_exposure(self, enabled: bool): pass
    def set_iso(self, iso_value: int): pass
    def release(self): pass

class OpenCVCam(CamBase):
    def __init__(self, index=0, width=None, height=None, fps=None):
        backend = cv2.CAP_DSHOW if platform.system()=="Windows" else 0
        self.cap = cv2.VideoCapture(index, backend)
        try:
            if platform.system() == "Windows":
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception: pass
        if width:  self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(width))
        if height: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps:    self.cap.set(cv2.CAP_PROP_FPS,          float(fps))
        try: self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception: pass
    def read(self): return self.cap.read()
    def set_auto_exposure(self, enabled: bool):
        try:
            if platform.system()=="Windows":
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if enabled else 0.25)
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if enabled else 1)
        except Exception: pass
    def set_iso(self, iso_value: int):
        try:
            if self.cap.set(cv2.CAP_PROP_ISO_SPEED, float(iso_value)): return
        except Exception: pass
        try:
            gain = max(1.0, iso_value/100.0)
            self.cap.set(cv2.CAP_PROP_GAIN, float(gain))
        except Exception: pass
    def release(self):
        try: self.cap.release()
        except Exception: pass

# ---- Picamera2 + Transform (libcamera) ----
try:
    from picamera2 import Picamera2
    HAVE_PICAM = True
    PICAM_IMPORT_ERROR = None
except Exception as e:
    HAVE_PICAM = False
    PICAM_IMPORT_ERROR = repr(e)

try:
    from libcamera import Transform
    HAVE_LIBCAM_TFM = True
except Exception:
    Transform = None
    HAVE_LIBCAM_TFM = False

class PiCam(CamBase):
    def __init__(self, size=(960,540), rotate=0, mirror=False):
        self.p = Picamera2()
        if Transform is not None:
            tfm = Transform(rotation=int(rotate), hflip=bool(mirror))
            conf = self.p.create_video_configuration(main={"size": size, "format": "BGR888"}, transform=tfm)
            self._sw_rotate = 0
            self._sw_mirror = False
        else:
            conf = self.p.create_video_configuration(main={"size": size, "format": "BGR888"})
            self._sw_rotate = int(rotate)
            self._sw_mirror = bool(mirror)
        self.p.configure(conf); self.p.start()
        self.auto = True; self.current_iso = 200; self._apply()
    def _apply(self):
        try:
            if self.auto:
                self.p.set_controls({"AeEnable": True})
            else:
                gain = max(1.0, self.current_iso/100.0)
                self.p.set_controls({"AeEnable": False, "AnalogueGain": float(gain)})
        except Exception: pass
    def read(self):
        f = self.p.capture_array()
        img = cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
        if self._sw_rotate or self._sw_mirror:
            img = apply_orientation(img, rotate=self._sw_rotate, mirror=self._sw_mirror)
        return True, img
    def set_auto_exposure(self, enabled: bool):
        self.auto = bool(enabled); self._apply()
    def set_iso(self, iso_value: int):
        self.current_iso = int(np.clip(iso_value, 50, 1600))
        if not self.auto: self._apply()
    def release(self):
        try: self.p.stop()
        except Exception: pass

def apply_orientation(img, rotate=0, mirror=False):
    if rotate == 90:   img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    elif rotate == 180: img = cv2.rotate(img, cv2.ROTATE_180)
    elif rotate == 270: img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if mirror: img = cv2.flip(img, 1)
    return img

def ensure_focal_from_fov(frame_w):
    global FOCAL_PX
    if USE_HFOV and FOCAL_PX is None and frame_w>0:
        FOCAL_PX = (0.5*frame_w) / math.tan(math.radians(HFOV_DEG*0.5))
def angle_cos(p0,p1,p2):
    d1, d2 = p0-p1, p2-p1
    return float(np.dot(d1,d2)/(np.linalg.norm(d1)*np.linalg.norm(d2)+1e-8))
def is_square(c, min_area=400, angle_tol=0.20, aspect_tol=0.20):
    area = cv2.contourArea(c)
    if area < min_area: return False
    approx = cv2.approxPolyDP(c, 0.02*cv2.arcLength(c,True), True)
    if len(approx)!=4 or not cv2.isContourConvex(approx): return False
    pts = approx.reshape(-1,2).astype(np.float32)
    max_cos = max(abs(angle_cos(pts[(i-1)%4], pts[i], pts[(i+1)%4])) for i in range(4))
    if max_cos > angle_tol: return False
    x,y,w,h = cv2.boundingRect(approx)
    if h==0: return False
    if abs((w/float(h))-1.0) > aspect_tol: return False
    return True
def approx_square_corners(contour):
    eps = 0.02*cv2.arcLength(contour, True)
    ap = cv2.approxPolyDP(contour, eps, True)
    if ap is None or len(ap)!=4: return None
    pts = ap.reshape(-1,2).astype(np.float32)
    c = np.mean(pts, axis=0)
    ang = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    return pts[np.argsort(ang)]
def side_length_pixels(pts):
    if pts is None or len(pts)!=4: return None
    d01=np.linalg.norm(pts[0]-pts[1]); d12=np.linalg.norm(pts[1]-pts[2])
    d23=np.linalg.norm(pts[2]-pts[3]); d30=np.linalg.norm(pts[3]-pts[0])
    return float((d01+d12+d23+d30)/4.0)
def distance_from_altitude(pixel, frame_shape, H_m):
    h,w = frame_shape[:2]; ensure_focal_from_fov(w)
    if FOCAL_PX is None or H_m is None: return None, None
    u,v = float(pixel[0]), float(pixel[1]); cx,cy = w/2.0, h/2.0; f = float(FOCAL_PX)
    x = (u-cx)/f; y=(v-cy)/f
    return H_m*math.sqrt(x*x+y*y+1.0), H_m*math.sqrt(x*x+y*y)
def estimate_distance_m_robust(contour, frame_shape):
    h,w = frame_shape[:2]; ensure_focal_from_fov(w)
    pts = approx_square_corners(contour)
    if pts is None: return None, None, None
    f = FOCAL_PX if FOCAL_PX else 800.0
    cx,cy = w/2.0, h/2.0
    K = np.array([[f,0,cx],[0,f,cy],[0,0,1]], dtype=np.float32)
    dist = np.zeros((5,1), np.float32)
    s = REAL_SIZE_M/2.0
    objp = np.array([[-s,-s,0],[s,-s,0],[s,s,0],[-s,s,0]], np.float32)
    imgp = pts.astype(np.float32)
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    Z=None
    ok,rvec,tvec = cv2.solvePnP(objp,imgp,K,dist,flags=flags)
    if ok: Z = float(np.linalg.norm(tvec))
    else:
        side_px = side_length_pixels(pts)
        if side_px and side_px>1: Z = (f*REAL_SIZE_M)/side_px
    side_px = side_length_pixels(pts)
    return Z, side_px, pts
def bbox_area_px2(contour):
    x,y,w,h = cv2.boundingRect(contour)
    return int(w*h)
def _border_touch_score(contour, frame_shape, margin=3):
    h,w = frame_shape[:2]; xs=contour[:,:,0]; ys=contour[:,:,1]
    touches = (xs<=margin).any() or (xs>=w-1-margin).any() or (ys<=margin).any() or (ys>=h-1-margin).any()
    return 0.0 if touches else 1.0
def _right_angle_score(pts):
    if pts is None or len(pts)!=4: return 0.0
    angs=[]
    for i in range(4):
        a=pts[(i-1)%4]; b=pts[i]; c=pts[(i+1)%4]
        v1=a-b; v2=c-b
        cos=np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)+1e-9)
        deg=np.degrees(np.arccos(np.clip(cos,-1,1))); angs.append(deg)
    err=np.mean([abs(90.0-d) for d in angs]); return float(max(0.0,1.0-(err/45.0)))
def _aspect_score_minrect(contour):
    (cx,cy),(w,h),rot = cv2.minAreaRect(contour)
    if w<=1 or h<=1: return 0.0
    r=max(w,h)/min(w,h); return float(max(0.0,1.0-(r-1.0)))
def _fill_scores(contour):
    area=max(cv2.contourArea(contour),1.0)
    x,y,w,h=cv2.boundingRect(contour); rect_area=max(w*h,1.0)
    extent=float(area/rect_area)
    hull=cv2.convexHull(contour); hull_area=max(cv2.contourArea(hull),1.0)
    solidity=float(area/hull_area)
    return extent, solidity
def compute_confidence(t, frame_shape):
    h,w=frame_shape[:2]
    vis=min(t.get("visible_frames",0)/max(lock_threshold,1),1.0)
    c=t.get("contour",None)
    if c is None: return round(float(0.4*vis),2)
    pts=approx_square_corners(c)
    area=cv2.contourArea(c)
    border=_border_touch_score(c,frame_shape)
    right90=_right_angle_score(pts)
    aspect=_aspect_score_minrect(c)
    extent,solidity=_fill_scores(c)
    area_norm=max(0.0,min(area/8000.0,1.0))
    cx,cy=t["cx"],t["cy"]
    offset=math.hypot(cx-w/2, cy-h/2)/(0.5*w)
    offset_score=float(max(0.4,1.0-0.6*offset))
    base=(0.15*vis+0.20*right90+0.15*aspect+0.20*extent+0.15*solidity+0.15*area_norm)
    conf=border*base*offset_score
    if area_norm<0.3 or right90<0.6 or extent<0.6: conf=min(conf,0.6)
    return round(float(max(min(conf,1.0),0.0)),2)
def get_current_gps(): return None, None
def get_altitude_m():  return None
def accept_by_range(distance_m, side_px):
    if distance_m is not None:
        if (ACCEPT_DIST_MIN_M is not None and distance_m < ACCEPT_DIST_MIN_M): return False,"too-near"
        if (ACCEPT_DIST_MAX_M is not None and distance_m > ACCEPT_DIST_MAX_M): return False,"too-far"
        return True,"ok"
    if side_px is not None and NEAR_REJECT_SIDE_PX is not None and side_px >= NEAR_REJECT_SIDE_PX:
        return False,"near-by-scale"
    return True,"ok"

def classify_quadrant(x, y, w, h, tol=20):
    cx, cy = w//2, h//2
    dx, dy = x - cx, y - cy
    if abs(dx) <= tol and abs(dy) <= tol: return "center"
    return f"{'bottom' if dy>0 else 'top'}-{'right' if dx>0 else 'left'}"

def _blend_rect(img, tl, br, alpha=0.45, color=(0,0,0)):
    x1,y1 = tl; x2,y2 = br
    x1 = max(0, min(x1, img.shape[1]-1))
    x2 = max(0, min(x2, img.shape[1]))
    y1 = max(0, min(y1, img.shape[0]-1))
    y2 = max(0, min(y2, img.shape[0]))
    if x2<=x1 or y2<=y1: return
    roi = img[y1:y2, x1:x2].copy()
    bg  = np.full_like(roi, color, dtype=np.uint8)
    cv2.addWeighted(bg, alpha, roi, 1-alpha, 0, roi)
    img[y1:y2, x1:x2] = roi

def draw_quadrant_grid(img):
    h, w = img.shape[:2]; cx, cy = w//2, h//2
    cv2.line(img, (cx, 0), (cx, h-1), (120,120,120), 1, cv2.LINE_AA)
    cv2.line(img, (0, cy), (w-1, cy), (120,120,120), 1, cv2.LINE_AA)

def count_contours(mask):
    cnt,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = len(cnt)
    squares = sum(1 for c in cnt if cv2.contourArea(c)>=MIN_AREA and is_square(c, MIN_AREA))
    return total, squares

def detect_targets(mask, min_area, tracked_list):
    contours,_=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current=[]
    for c in contours:
        if cv2.contourArea(c) >= min_area and is_square(c, MIN_AREA):
            M=cv2.moments(c)
            if M["m00"]!=0:
                cx,cy=int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                current.append({"cx":cx,"cy":cy,"locked":False,
                                "visible_frames":1,"lost_frames":0,"contour":c})
    for det in current:
        matched=False
        for t in tracked_list:
            if abs(det["cx"]-t["cx"])<position_tolerance and abs(det["cy"]-t["cy"])<position_tolerance:
                t.update({"cx":det["cx"], "cy":det["cy"], "contour":det["contour"]})
                t["visible_frames"]+=1; t["lost_frames"]=0
                matched=True; break
        if not matched: tracked_list.append(det)
    for t in tracked_list:
        if all(abs(t["cx"]-d["cx"])>position_tolerance or abs(t["cy"]-d["cy"])>position_tolerance for d in current):
            t["lost_frames"]+=1
    tracked_list[:]=[t for t in tracked_list if t["lost_frames"]<=lost_frame_threshold]

def build_detection_info(color_name, target_dict, frame_shape):
    cx, cy = int(target_dict["cx"]), int(target_dict["cy"])
    contour = target_dict.get("contour", None)
    area_px2 = bbox_area_px2(contour) if contour is not None else 0
    H = get_altitude_m()
    dist_alt, _ = (None, None)
    if H is not None: dist_alt, _ = distance_from_altitude((cx, cy), frame_shape, H)
    dist_size, side_px, _ = (None, None, None)
    if dist_alt is None and contour is not None:
        dist_size, side_px, _ = estimate_distance_m_robust(contour, frame_shape)
    chosen_dist = dist_alt if dist_alt is not None else dist_size
    q = classify_quadrant(cx, cy, frame_shape[1], frame_shape[0])
    info = {
        "type": "detection",
        "detected": True,
        "color": color_name,
        "target_type": f"{color_name}_4x4",
        "pixel": [cx, cy],
        "bbox_area": int(area_px2),
        "distance_m": round(float(chosen_dist), 2) if chosen_dist is not None else None,
        "confidence": compute_confidence(target_dict, frame_shape),
        "quadrant": q
    }
    debug = {"side_px": side_px, "FOCAL_PX": FOCAL_PX, "HFOV_DEG": HFOV_DEG}
    return info, debug

def run(cam_backend="opencv", cam_index=0, port=8080, ae=True, iso=200,
        width=None, height=None, fps=None, rotate=0, mirror=False):
    global prev_mask_blue, prev_mask_red, tracked_blue, tracked_red

    if cam_backend == "opencv" and platform.system()=="Linux" and shutil.which("rpicam-hello"):
        cam_backend = "picam"

    if cam_backend=="picam":
        if not HAVE_PICAM:
            print("Picamera2 import edilemedi. Kurulum:\n  sudo apt update && sudo apt install -y python3-picamera2 rpicam-apps")
            print(f"Import error: {PICAM_IMPORT_ERROR}")
            sys.exit(1)
        cam = PiCam(size=((width or 960), (height or 540)), rotate=rotate, mirror=mirror)
        apply_sw_orientation = False
    else:
        cam = OpenCVCam(index=cam_index, width=width, height=height, fps=fps)
        apply_sw_orientation = True

    cam.set_auto_exposure(bool(ae))
    if not ae: cam.set_iso(int(iso))

    server = None
    if port and port > 0:
        server = start_streaming_server(port=port, target_fps=STREAM_TARGET_FPS,
                                     max_width=STREAM_MAX_WIDTH,
                                     jpeg_quality=STREAM_JPEG_QUALITY)

    print(f"Backend: {cam_backend} | Index: {cam_index} | AE: {ae} | ISO: {iso} | Display: False (headless)")

    prev_mask_blue = prev_mask_red = None
    tracked_blue, tracked_red = [], []
    t_last_log = 0.0
    t_last_heartbeat = 0.0
    fps_smooth = 0.0

    try:
        while True:
            t_loop = time.time()
            ok, frame = cam.read()
            if not ok or frame is None:
                time.sleep(0.01); continue

            if apply_sw_orientation:
                frame = apply_orientation(frame, rotate=rotate, mirror=mirror)

            ensure_focal_from_fov(frame.shape[1])
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, w = frame.shape[:2]
            ref_x, ref_y = w//2, h//2

            disp = frame.copy()
            draw_quadrant_grid(disp)
            cv2.circle(disp, (ref_x, ref_y), 4, (0, 255, 0), -1)

            mask_blue_hsv = cv2.inRange(hsv, (BLUE_H_LO, BLUE_S_LO, BLUE_V_LO), (BLUE_H_HI, 255, 255))
            white_spec = cv2.inRange(hsv, (0, 0, WHITE_V_LO), (179, WHITE_S_HI, 255))
            b,g,r = cv2.split(frame); rg_max = cv2.max(r,g); b_dom = cv2.subtract(b, rg_max)
            _, b_dom_bin = cv2.threshold(b_dom, B_DOM_MARGIN, 255, cv2.THRESH_BINARY)
            mask_blue = cv2.bitwise_and(mask_blue_hsv, cv2.bitwise_not(white_spec))
            mask_blue = cv2.bitwise_and(mask_blue, b_dom_bin)
            mask_red  = (cv2.inRange(hsv, (0,180,150), (10,255,255)) |
                         cv2.inRange(hsv, (160,100,100), (179,255,255)))

            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN,  kernel, 1)
            mask_red  = cv2.morphologyEx(mask_red,  cv2.MORPH_OPEN,  kernel, 1)

            if prev_mask_blue is not None and alpha_mask < 1:
                mask_blue = cv2.addWeighted(mask_blue, alpha_mask, prev_mask_blue, 1-alpha_mask, 0).astype(np.uint8)
            prev_mask_blue = mask_blue.copy()
            if prev_mask_red is not None and alpha_mask < 1:
                mask_red = cv2.addWeighted(mask_red, alpha_mask, prev_mask_red, 1-alpha_mask, 0).astype(np.uint8)
            prev_mask_red = mask_red.copy()

            blue_total, blue_sq = count_contours(mask_blue)
            red_total,  red_sq  = count_contours(mask_red)
            blue_nz = int(np.count_nonzero(mask_blue))
            red_nz  = int(np.count_nonzero(mask_red))

            detect_targets(mask_blue, MIN_AREA, tracked_blue)
            detect_targets(mask_red, 1000,       tracked_red)

            detection_json_to_show = None
            for color_name, tracked_list in [("blue", tracked_blue), ("red", tracked_red)]:
                for t in tracked_list:
                    if not t["locked"] and t["visible_frames"]>=lock_threshold: t["locked"]=True
                    if not t["locked"]: continue
                    cx, cy = t["cx"], t["cy"]
                    cv2.drawContours(disp,[t["contour"]],-1,(0,0,255),2)
                    cv2.line(disp, (ref_x, ref_y), (cx, cy), (0, 255, 0), 2, cv2.LINE_AA)
                    info, dbg = build_detection_info(color_name, t, frame.shape)
                    dist_c = math.hypot(cx - ref_x, cy - ref_y)
                    max_dist = math.hypot(ref_x, ref_y)
                    score = int(100 * (1 - (dist_c / max_dist))); score = max(0, min(100, score))
                    info["center_score"] = score
                    acc, reason = accept_by_range(info.get("distance_m"), dbg.get("side_px"))
                    info["accept"] = bool(acc); info["reject_reason"] = None if acc else reason
                    txt = f"Score:{score}  Q:{info['quadrant']}"
                    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    lx = min(max(cx + 10, 0), w - tw - 10)
                    ly = min(max(cy + 20, th + 10), h - 10)
                    _blend_rect(disp, (lx-6, ly-20), (lx+tw+6, ly+8), 0.45, (0,0,0))
                    cv2.putText(disp, txt, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
                    detection_json_to_show = info

            dt = max(1e-6, time.time() - t_loop)
            inst_fps = 1.0 / dt
            fps_smooth = inst_fps if fps_smooth == 0 else (0.9*fps_smooth + 0.1*inst_fps)
            now = time.time()

            stream_fps = None
            if 'server' in locals() and server is not None and hasattr(server, "_stream_cnt"):
                elapsed = now - getattr(server, "_stream_t0", now)
                if elapsed > 0:
                    stream_fps = round(server._stream_cnt / elapsed, 1)
                server._stream_cnt = 0; server._stream_t0 = now

            if (now - t_last_heartbeat) > 2.0:
                hb = {
                    "type": "heartbeat",
                    "fps": round(fps_smooth, 1),
                    "stream_fps": stream_fps,
                    "frame": {"w": w, "h": h},
                    "masks": {"blue_nonzero": blue_nz, "red_nonzero": red_nz},
                    "contours": {
                        "blue_total": blue_total, "blue_square": blue_sq,
                        "red_total": red_total,   "red_square": red_sq
                    },
                    "tracked": {"blue": len(tracked_blue), "red": len(tracked_red)},
                    "focal_px": None if FOCAL_PX is None else round(float(FOCAL_PX), 2)
                }
                print(json.dumps(hb, ensure_ascii=False))
                t_last_heartbeat = now

            if detection_json_to_show is not None and (now - t_last_log) > 0.3:
                print(json.dumps(detection_json_to_show, ensure_ascii=False))
                t_last_log = now

            if 'server' in locals() and server is not None:
                server.current_frame = disp

    except KeyboardInterrupt:
        pass
    finally:
        try: cam.release()
        except Exception: pass
        try:
            if 'server' in locals() and server is not None:
                server.shutdown(); server.server_close()
        except Exception: pass
        print("Kapatıldı.")

def parse_args():
    prefer_picam = (platform.system()=="Linux" and (HAVE_PICAM or shutil.which("rpicam-hello")))
    default_backend = "picam" if prefer_picam else "opencv"
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, default=int(os.getenv("CAM_INDEX", "0")))
    p.add_argument("--port",  type=int, default=int(os.getenv("PORT", "8080")))
    p.add_argument("--backend", choices=["opencv","picam"], default=os.getenv("BACKEND", default_backend))
    p.add_argument("--ae", action="store_true")
    p.add_argument("--no-ae", dest="ae", action="store_false"); p.set_defaults(ae=True)
    p.add_argument("--iso", type=int, default=int(os.getenv("ISO","200")))
    p.add_argument("--width",  type=int, default=None)
    p.add_argument("--height", type=int, default=None)
    p.add_argument("--fps",    type=int, default=None)
    p.add_argument("--rotate", type=int, choices=[0,90,180,270], default=int(os.getenv("ROTATE","0")))
    p.add_argument("--mirror", action="store_true")
    p.add_argument("--no-mirror", dest="mirror", action="store_false")
    p.set_defaults(mirror=False)
    return p.parse_args()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    args = parse_args()
    run(
        cam_backend=args.backend,
        cam_index=args.index,
        port=args.port,
        ae=args.ae,
        iso=args.iso,
        width=args.width,
        height=args.height,
        fps=args.fps,
        rotate=args.rotate,
        mirror=args.mirror
    )
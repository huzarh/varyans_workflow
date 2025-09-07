#!/usr/bin/env python3
"""
Sistem konfigürasyonu ve sabitler
"""
from dataclasses import dataclass
from typing import Tuple

@dataclass
class CameraConfig:
    """Kamera konfigürasyonu"""
    width: int = 1280
    height: int = 720
    fps: int = 30
    hfov_deg: float = 66.0
    real_size_m: float = 0.05  # 5 cm kare

@dataclass
class DetectionConfig:
    """Hedef algılama konfigürasyonu"""
    center_tolerance: int = 40
    position_tolerance: int = 60
    lock_threshold: int = 3
    lost_frame_threshold: int = 10
    min_area: int = 300
    max_area_blue: int = 20000
    max_area_red: int = 20000
    print_every_n: int = 10
    action_cooldown_sec: float = 1.5

@dataclass
class ServoConfig:
    """Servo konfigürasyonu"""
    servo_id: int = 7
    normal_pwm: int = 1500
    blue_cargo_pwm: int = 1600
    red_cargo_pwm: int = 800
    servo_wait_sec: float = 2.0

@dataclass
class MissionConfig:
    """Görev konfigürasyonu"""
    max_cargo: int = 2
    rtl_mode_id: int = 6
    guided_mode_id: int = 15

@dataclass
class VelocityConfig:
    """Velocity kontrol konfigürasyonu"""
    max_vx: float = 1.5
    max_vy: float = 2.0
    max_vz: float = 1.5
    approach_factor: float = 0.15
    direction_factor_x: float = 1.5
    direction_factor_y: float = 1.0

# Global konfigürasyon instance'ları
camera_config = CameraConfig()
detection_config = DetectionConfig()
servo_config = ServoConfig()
mission_config = MissionConfig()
velocity_config = VelocityConfig()

# MAVLink bağlantı ayarları
MAVLINK_CONNECTION = 'udp:127.0.0.1:14540'
MAVLINK_TIMEOUT = 5

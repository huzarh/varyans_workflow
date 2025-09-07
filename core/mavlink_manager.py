#!/usr/bin/env python3
"""
MAVLink bağlantı yönetimi için merkezi sınıf
"""
from pymavlink import mavutil
import time
from typing import Optional

class MAVLinkManager:
    """MAVLink bağlantılarını yöneten merkezi sınıf"""
    
    def __init__(self, connection_string: str = 'udp:127.0.0.1:14540'):
        self.connection_string = connection_string
        self.master: Optional[mavutil.mavlink_connection] = None
        self.is_connected = False
    
    def connect(self, timeout: int = 5) -> bool:
        """MAVLink bağlantısı kur"""
        try:
            if self.master is None:
                self.master = mavutil.mavlink_connection(self.connection_string)
            
            self.master.wait_heartbeat(timeout=timeout)
            self.is_connected = True
            print(f"✅ MAVLink bağlandı! System: {self.master.target_system}, Component: {self.master.target_component}")
            return True
        except Exception as e:
            print(f"❌ MAVLink bağlantısı başarısız: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """MAVLink bağlantısını kapat"""
        if self.master:
            try:
                self.master.close()
            except:
                pass
            finally:
                self.master = None
                self.is_connected = False
    
    def set_mode(self, mode_id: int, mode_name: str) -> bool:
        """Mod değiştir"""
        if not self.is_connected or not self.master:
            print("❌ MAVLink bağlantısı yok!")
            return False
        
        try:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            time.sleep(2)
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if hb and hb.custom_mode == mode_id:
                print(f"✅ {mode_name} moduna geçildi")
                return True
            else:
                print(f"❌ {mode_name} moduna geçilemedi")
                return False
        except Exception as e:
            print(f"❌ Mod değiştirme hatası: {e}")
            return False
    
    def send_velocity_command(self, vx: float, vy: float, vz: float):
        """Velocity komutu gönder"""
        if not self.is_connected or not self.master:
            print("❌ MAVLink bağlantısı yok!")
            return False
        
        try:
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                int(0b0000111111000111),  # sadece velocity enable
                0, 0, 0,  # position (kullanılmıyor)
                vx, vy, vz,  # velocity
                0, 0, 0,     # acceleration
                0, 0         # yaw/yaw_rate
            )
            return True
        except Exception as e:
            print(f"❌ Velocity komutu hatası: {e}")
            return False
    
    def send_servo_command(self, servo_id: int, pwm_value: int):
        """Servo komutu gönder"""
        if not self.is_connected or not self.master:
            print("❌ MAVLink bağlantısı yok!")
            return False
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, servo_id, pwm_value, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Servo komutu hatası: {e}")
            return False
    
    def __enter__(self):
        """Context manager giriş"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager çıkış"""
        self.disconnect()

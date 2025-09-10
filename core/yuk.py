#!/usr/bin/env python3
"""
YUK Module - Servo 7 Target Control
Servo 7 için hedef kontrol fonksiyonları
"""
from pymavlink import mavutil
import time

# Global connection variable
_master = None

def initialize_connection():
    """MAVLink bağlantısını başlat"""
    global _master
    
    if _master is None:
        print("🔗 MAVLink bağlantısı başlatılıyor...")
        _master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        _master.wait_heartbeat()
        print(f"✅ Bağlantı kuruldu! System: {_master.target_system}")
    
    return _master

def _set_servo7_pwm(pwm_value, target_name):
    """Servo 7'yi belirtilen PWM değerine ayarla"""
    master = initialize_connection()
    
    print(f"🎯 {target_name} -> Servo 7: {pwm_value} PWM")
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,          # confirmation
        7,          # param1: servo 7
        pwm_value,  # param2: PWM value
        0, 0, 0, 0, 0  # param3-7: unused
    )
    
    # ACK kontrol
    start_time = time.time()
    while time.time() - start_time < 2:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"✅ {target_name} komutu kabul edildi")
                return True
            else:
                print(f"❌ {target_name} komutu reddedildi")
                return False
    
    print(f"⚠️ {target_name} timeout")
    return False

def servo_red_target():
    """
    Kırmızı hedef - Servo 7'yi 2000 PWM'e ayarla
    Returns: bool - Komut başarılı olursa True
    """
    return _set_servo7_pwm(2000, "RED TARGET")

def servo_blue_target():
    """
    Mavi hedef - Servo 7'yi 1000 PWM'e ayarla  
    Returns: bool - Komut başarılı olursa True
    """
    return _set_servo7_pwm(1000, "BLUE TARGET")
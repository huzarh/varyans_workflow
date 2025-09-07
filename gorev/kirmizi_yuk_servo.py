#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("🔴 Kırmızı Yük Servo 7 Testi")

# Bağlan
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("✅ MAVLink bağlandı!")

def move_servo7(pwm):
    """Servo 7'i hareket ettir (Kırmızı yük)"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 7, pwm, 0, 0, 0, 0, 0
    )
    print(f"🔴 Kırmızı yük Servo 7: {pwm} PWM")
    time.sleep(1)

def drop_red_cargo():
    """Kırmızı yükü bırak"""
    print("🔴 Kırmızı yük bırakılıyor...")
    move_servo7(800)   # Yük bırakma pozisyonu
    time.sleep(2)
    move_servo7(1500)  # Normal pozisyon
    print("✅ Kırmızı yük bırakıldı!")

def reset_red_cargo():
    """Kırmızı yük servosunu sıfırla"""
    print("🔴 Kırmızı yük servosu sıfırlanıyor...")
    move_servo7(1500)  # Normal pozisyon

print("🔄 Kırmızı yük servo testi başlıyor...")

try:
    # Test sekansı
    move_servo7(1000)  # Sol
    move_servo7(1500)  # Merkez
    move_servo7(2000)  # Sağ
    move_servo7(1500)  # Merkez
    
    # Yük bırakma testi
    drop_red_cargo()
    
    print("✅ Kırmızı yük testi tamamlandı!")
    
except KeyboardInterrupt:
    print("\n🛑 Durduruluyor...")
    
finally:
    master.close()
    print("👋 Kırmızı yük servo testi bitti!")

if __name__ == "__main__":
    # Fonksiyonları dışarıdan kullanılabilir yap
    pass

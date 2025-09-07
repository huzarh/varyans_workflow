#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("🔵 Mavi Yük Servo 7 Testi")

# Bağlan
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("✅ MAVLink bağlandı!")

def move_servo7(pwm):
    """Servo 7'i hareket ettir (Mavi yük)"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 7, pwm, 0, 0, 0, 0, 0
    )
    print(f"🔵 Mavi yük Servo 7: {pwm} PWM")
    time.sleep(1)

def drop_blue_cargo():
    """Mavi yükü bırak"""
    print("🔵 Mavi yük bırakılıyor...")
    move_servo7(1600)  # Yük bırakma pozisyonu
    time.sleep(2)
    move_servo7(1500)  # Normal pozisyon
    print("✅ Mavi yük bırakıldı!")

def reset_blue_cargo():
    """Mavi yük servosunu sıfırla"""
    print("🔵 Mavi yük servosu sıfırlanıyor...")
    move_servo7(1500)  # Normal pozisyon

print("🔄 Mavi yük servo testi başlıyor...")

try:
    # Test sekansı
    move_servo7(1200)  # Sol
    move_servo7(1500)  # Merkez
    move_servo7(1800)  # Sağ
    move_servo7(1500)  # Merkez
    
    # Yük bırakma testi
    drop_blue_cargo()
    
    print("✅ Mavi yük testi tamamlandı!")
    
except KeyboardInterrupt:
    print("\n🛑 Durduruluyor...")
    
finally:
    master.close()
    print("👋 Mavi yük servo testi bitti!")

if __name__ == "__main__":
    # Fonksiyonları dışarıdan kullanılabilir yap
    pass

#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("🎯 SERVO 7 TEST SCRIPT")
print("=" * 25)

# Pixhawk'a bağlan
print("🔗 Bağlanıyor...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"✅ Bağlandı! System: {master.target_system}")

def set_servo7(pwm_value):
    """Servo 7'yi belirtilen PWM değerine ayarla"""
    print(f"🎛️ Servo 7 -> {pwm_value} PWM")
    
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
                print(f"✅ Servo 7 komutu kabul edildi")
                return True
            else:
                print(f"❌ Servo 7 komutu reddedildi")
                return False
    
    print(f"⚠️ Servo 7 timeout")
    return False

try:
    print("\n🚀 Servo 7 Test Başlıyor...")
    
    while True:
        print("\n📋 SERVO 7 KOMUTLARI:")
        print("1: 1000 PWM (Minimum)")
        print("2: 2000 PWM (Maximum)")
        print("3: 1500 PWM (Merkez)")
        print("q: Çıkış")
        
        choice = input("\nSeçim: ").strip().lower()
        
        if choice == '1':
            set_servo7(1000)
            
        elif choice == '2':
            set_servo7(2000)
            
        elif choice == '3':
            set_servo7(1500)
            
        elif choice == 'q':
            break
            
        else:
            print("❌ Geçersiz seçim!")

except KeyboardInterrupt:
    print("\n🛑 Çıkılıyor...")

finally:
    # Servo 7'yi merkeze getir
    print("🔄 Servo 7 merkeze getiriliyor...")
    set_servo7(1500)
    time.sleep(0.5)
    master.close()
    print("✅ Test tamamlandı!")

print("👋 Servo 7 test bitti!")
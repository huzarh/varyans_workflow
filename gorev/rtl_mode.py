#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("🏠 RTL (Return to Launch) Modu")

def switch_to_rtl():
    """Drone'u RTL moduna geçir"""
    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ MAVLink bağlandı! System: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız: {e}")
        return False

    try:
        # RTL moduna geçiş
        print("🏠 RTL moduna geçiş yapılıyor...")
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6  # RTL mode ID
        )
        
        # Mod değişimini kontrol et
        time.sleep(2)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb and hb.custom_mode == 6:
            print("✅ RTL moduna başarıyla geçildi!")
            print("🏠 Drone eve dönüyor...")
            return True
        else:
            print("❌ RTL moduna geçilemedi")
            return False
            
    except Exception as e:
        print(f"❌ RTL mod geçiş hatası: {e}")
        return False
    finally:
        master.close()

def test_rtl_mode():
    """RTL modunu test et"""
    print("🧪 RTL mod testi başlatılıyor...")
    
    success = switch_to_rtl()
    
    if success:
        print("✅ RTL mod testi başarılı!")
    else:
        print("❌ RTL mod testi başarısız!")
    
    return success

if __name__ == "__main__":
    test_rtl_mode()

#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("AUTO -> GUIDED test scripti")

# MAVLink bağlantısı (simülasyonda genelde 14540 portu)
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print(f"Baglandi! GUİDED ----------------- System: {master.target_system}, Component: {master.target_component}")

# Mod değiştirme fonksiyonu
def set_mode(mode_id, mode_name):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(2)
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if hb and hb.custom_mode == mode_id:
        print(f"✅ {mode_name} moduna gecildi")
    else:
        print(f"❌ {mode_name} moduna gecilemedi")

# AUTO → GUIDED
print("AUTO moddan GUIDED moda gecis deneniyor...")
set_mode(15, "GUIDED")

master.close()
print("Baglanti kapatildi!")

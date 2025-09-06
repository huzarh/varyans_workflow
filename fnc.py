def test_mavlink_connection():
    from pymavlink import mavutil
    import sys
    import time

    print("MAVLink bağlantısı kuruluyor...")

    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ MAVLink bağlandı! System: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız: {e}")
        sys.exit(1)

    try:
        print("📤 TEST: STATUSTEXT mesajı gönderiliyor...")
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_NOTICE,
            b"Baglanti testi: Varyans sistem basladi."
        )
        print("✅ STATUSTEXT mesajı gönderildi.")
    except Exception as e:
        print(f"❌ MAVLink mesaj gönderimi hatası: {e}")
        sys.exit(1)

    return master

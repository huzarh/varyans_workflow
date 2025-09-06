def test_mavlink_connection():
    from pymavlink import mavutil
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

def switch_to_guided_mode():
    """
    MAVLink bağlantısı kurar ve drone'u GUIDED moduna geçirir.
    Returns:
        master: MAVLink bağlantı objesi
    """
    from pymavlink import mavutil
    import time
    import sys

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
            return True
        else:
            print(f"❌ {mode_name} moduna gecilemedi")
            return False

    # AUTO → GUIDED
    print("AUTO moddan GUIDED moda gecis deneniyor...")
    success = set_mode(15, "GUIDED")
    
    if success:
        print("✅ GUIDED moda başarıyla geçildi!")
    else:
        print("❌ GUIDED moda geçiş başarısız!")
    
    return master

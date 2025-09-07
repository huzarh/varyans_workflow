
def test_mavlink_connection():
    from pymavlink import mavutil
    import time

    print("MAVLink bağlantısı kuruluyor -----------------GUİDED ...")

    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ MAVLink bağlandı! System: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız: {e}")
        sys.exit(1)

    try:
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
    except Exception as e:
        print(f"❌ MAVLink mesaj gönderimi hatası: {e}")
        sys.exit(1)

    return master

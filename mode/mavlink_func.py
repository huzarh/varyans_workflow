def switch_to_guided(connection: str = 'udp:127.0.0.1:14540', timeout: int = 5) -> bool:
    from pymavlink import mavutil

    master = None
    try:
        master = mavutil.mavlink_connection(connection)
        master.wait_heartbeat(timeout=timeout)
        print(f"✅ Bağlandı: sys={master.target_system}, comp={master.target_component}")

        # GUIDED moda geçiş
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            15  # GUIDED
        )

        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and getattr(hb, 'custom_mode', -1) == 15:
            print("✅ GUIDED moda geçildi")
            return True
        print("❌ GUIDED moda geçilemedi (heartbeat doğrulanamadı)")
        return False

    except Exception as e:
        print(f"❌ Hata: {e}")
        return False

    finally:
        if master:
            try:
                master.close()
            except Exception:
                pass

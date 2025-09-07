def switch_to_guided(connection: str = 'udp:127.0.0.1:14540', timeout: int = 5) -> bool:
    """GUIDED moda güvenli geçiş.
    Returns True on success, False otherwise.
    """
    from pymavlink import mavutil
    import time

    print("🔌 MAVLink'e bağlanılıyor (GUIDED)...")

    master = None
    try:
        master = mavutil.mavlink_connection(connection)
        master.wait_heartbeat(timeout=timeout)
        print(f"✅ Bağlandı: sys={master.target_system}, comp={master.target_component}")
    except Exception as e:
        print(f"❌ Bağlantı hatası: {e}")
        if master:
            try: master.close()
            except: pass
        return False

    try:
        print("▶️ GUIDED moda geçiş deneniyor...")
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            15
        )
        time.sleep(2)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb and getattr(hb, 'custom_mode', None) == 15:
            print("✅ GUIDED moda geçildi")
            return True
        else:
            print("❌ GUIDED moda geçilemedi (heartbeat doğrulanamadı)")
            return False
    except Exception as e:
        print(f"❌ GUIDED geçiş hatası: {e}")
        return False
    finally:
        try:
            master.close()
        except Exception:
            pass

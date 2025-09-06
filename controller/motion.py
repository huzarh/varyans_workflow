#!/usr/bin/env python3
def planning():
    from pymavlink import mavutil
    import time

    print("🚁 Servo 5 PWM Testi")

    # Bağlan
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    master.wait_heartbeat()
    print("✅ Bağlandı!")

    def move_servo5(pwm):
        """Servo 5'i hareket ettir"""
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0, 5, pwm, 0, 0, 0, 0, 0
        )
        print(f"Servo 5: {pwm} PWM")
        time.sleep(1)

    print("🔄 Servo 5 test başlıyor...")

    try:
        move_servo5(1200)  # Sol
        move_servo5(1500)  # Merkez
        move_servo5(1800)  # Sağ
        move_servo5(1500)  # Merkez
        
        print("✅ Test tamamlandı!")
        
    except KeyboardInterrupt:
        print("\n🛑 Durduruluyor...")
        
    finally:
        master.close()
        print("👋 Bitti!")
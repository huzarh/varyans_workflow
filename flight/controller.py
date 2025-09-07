#!/usr/bin/env python3
from core.state_manager import StateManager

def guided_approach_velocity(state: StateManager):
    from pymavlink import mavutil
    import time

    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ --------> controller: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız CONTROLLLER: {e}")
    while True:
        if state.state["target_detected"]:
            target = state.state["target_info"]
            distance = target["distance"]

            # küçük velocity komutu (yavaşça yaklaş)
            vx = min(2.0, distance * 0.2)  # ileri hız
            vy = 0
            vz = 0

            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                int(0b0000111111000111),  # sadece velocity enable
                0, 0, 0,
                vx, vy, vz,  # velocity
                0, 0, 0,     # accel
                0, 0         # yaw/yaw_rate
            )
            print(f"➡️ Velocity control: vx={vx}")
            time.sleep(0.1)

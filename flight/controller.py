#!/usr/bin/env python3
from core.state_manager import StateManager

def guided_approach_velocity(state: StateManager):
    print("tttttee---------------------eeesttt")
    from pymavlink import mavutil
    import time

    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ --------> controller: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız CONTROLLLER: {e}")
        return
    
    if state.state["target_detected"]:
        target = state.state["target_info"]
        # Use bbox_area as a proxy for distance (larger area = closer)
        bbox_area = target.get("bbox_area", 0)
        distance = max(0.5, 10.0 - (bbox_area / 1000.0))  # Convert area to distance estimate

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
        print(f"➡️ Velocity control: vx={vx:.2f}, distance_est={distance:.2f}")
    else:
        print("No target detected, no velocity command sent")
    
    master.close()

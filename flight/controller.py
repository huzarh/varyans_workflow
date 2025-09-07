#!/usr/bin/env python3
from core.state_manager import StateManager

def guided_approach_velocity(state: StateManager):
    print("🎯 Hedef yönlendirme kontrolü başlatılıyor...")
    from pymavlink import mavutil
    import time

    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"✅ MAVLink bağlandı! System: {master.target_system}, Component: {master.target_component}")
    except Exception as e:
        print(f"❌ MAVLink bağlantısı başarısız: {e}")
        return
    
    if state.state["target_detected"]:
        target = state.state["target_info"]
        cx = target.get("cx", 0)
        cy = target.get("cy", 0)
        bbox_area = target.get("bbox_area", 0)
        
        # Görüntü boyutları (remzi.py'de 1280x720 kullanılıyor)
        frame_width = 1280
        frame_height = 720
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # Hedefin merkezden uzaklığını hesapla
        offset_x = cx - center_x
        offset_y = cy - center_y
        
        # Yön kontrolü için velocity hesapla
        # Pozitif değerler: sağ, aşağı, ileri
        # Negatif değerler: sol, yukarı, geri
        
        # Yatay yön kontrolü (vy - sağ/sol hareket)
        if abs(offset_x) > 50:  # 50 pixel tolerans
            vy = -offset_x / center_x * 1.5  # Normalize et ve hız sınırla
            vy = max(-2.0, min(2.0, vy))  # -2 ile +2 arasında sınırla
        else:
            vy = 0  # Merkezde ise yatay hareket yok
        
        # Dikey yön kontrolü (vz - yukarı/aşağı hareket)
        if abs(offset_y) > 50:  # 50 pixel tolerans
            vz = offset_y / center_y * 1.0  # Normalize et ve hız sınırla
            vz = max(-1.5, min(1.5, vz))  # -1.5 ile +1.5 arasında sınırla
        else:
            vz = 0  # Merkezde ise dikey hareket yok
        
        # İleri hareket (vx) - hedefe yaklaşma
        distance = max(0.5, 10.0 - (bbox_area / 1000.0))  # bbox_area'dan mesafe tahmini
        vx = min(1.5, distance * 0.15)  # Yavaşça yaklaş
        
        # Hedef merkeze yakınsa dur
        if abs(offset_x) < 30 and abs(offset_y) < 30:
            vx = 0.2  # Çok yavaş ileri
            vy = 0
            vz = 0
            print("🎯 Hedef merkeze yakın - yavaş yaklaşma")
        
        # Velocity komutunu gönder
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            int(0b0000111111000111),  # sadece velocity enable
            0, 0, 0,  # position (kullanılmıyor)
            vx, vy, vz,  # velocity (ileri, sağ/sol, yukarı/aşağı)
            0, 0, 0,     # acceleration (kullanılmıyor)
            0, 0         # yaw/yaw_rate (kullanılmıyor)
        )
        
        # Yön bilgilerini yazdır
        direction = ""
        if offset_x > 50:
            direction += "SAĞ "
        elif offset_x < -50:
            direction += "SOL "
        if offset_y > 50:
            direction += "AŞAĞI "
        elif offset_y < -50:
            direction += "YUKARI "
        if abs(offset_x) <= 50 and abs(offset_y) <= 50:
            direction = "MERKEZ"
        
        print(f"🎯 Hedef: {direction} | Offset: ({offset_x:.0f}, {offset_y:.0f}) | Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
        
    else:
        print("❌ Hedef algılanmadı - hareket durduruluyor")
        # Hedef yoksa dur
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            int(0b0000111111000111),
            0, 0, 0,
            0, 0, 0,  # Tüm velocity'ler 0
            0, 0, 0,
            0, 0
        )
    
    master.close()

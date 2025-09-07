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
        target_type = target.get("type", "unknown")
        
        # Görüntü boyutları (remzi.py'de 1280x720 kullanılıyor)
        frame_width = 1280
        frame_height = 720
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # Hedefin merkezden uzaklığını hesapla
        offset_x = cx - center_x
        offset_y = cy - center_y
        total_offset = (offset_x**2 + offset_y**2)**0.5
        
        # 40 pixel tolerans ile merkez kontrolü
        CENTER_TOLERANCE = 40
        
        if total_offset <= CENTER_TOLERANCE:
            # Hedef merkeze geldi - yük bırakma zamanı!
            print(f"🎯 HEDEF MERKEZE GELDİ! Yük bırakma başlatılıyor...")
            print(f"📍 Hedef türü: {target_type}")
            print(f"📍 Merkez offset: {total_offset:.1f} pixel (tolerans: {CENTER_TOLERANCE})")
            
            # Yük bırakma fonksiyonu
            def drop_cargo():
                try:
                    if "blue" in target_type.lower():
                        print("🔵 Mavi yük bırakılıyor...")
                        master.mav.command_long_send(
                            master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0, 7, 1600, 0, 0, 0, 0, 0  # Mavi yük PWM: 1600
                        )
                        print("🔵 Mavi yük servosu 1600 PWM'e ayarlandı")
                        
                    elif "red" in target_type.lower():
                        print("🔴 Kırmızı yük bırakılıyor...")
                        master.mav.command_long_send(
                            master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0, 7, 800, 0, 0, 0, 0, 0   # Kırmızı yük PWM: 800
                        )
                        print("🔴 Kırmızı yük servosu 800 PWM'e ayarlandı")
                    
                    else:
                        print(f"⚠️ Bilinmeyen hedef türü: {target_type}")
                        return
                    
                    # 2 saniye bekle
                    time.sleep(2)
                    
                    # Servoyu normal pozisyona döndür
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, 7, 1500, 0, 0, 0, 0, 0  # Normal pozisyon
                    )
                    print("✅ Servo normal pozisyona döndürüldü (1500 PWM)")
                    
                    # Hedefi temizle (bir kez bırakıldı)
                    state.clear_target()
                    print("🎯 Hedef temizlendi - yeni hedef aranacak")
                    
                except Exception as e:
                    print(f"❌ Yük bırakma hatası: {e}")
            
            # Yük bırak
            drop_cargo()
            
            # Merkeze geldiğinde dur
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                int(0b0000111111000111),
                0, 0, 0,
                0, 0, 0,  # Tüm velocity'ler 0 - dur
                0, 0, 0,
                0, 0
            )
            print("🛑 Hedef merkeze geldi - drone durduruldu")
            
        else:
            # Hedef henüz merkeze gelmedi - yönlendirme devam et
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
                direction = "MERKEZE YAKIN"
            
            print(f"🎯 Hedef: {direction} | Offset: ({offset_x:.0f}, {offset_y:.0f}) | Merkez mesafesi: {total_offset:.1f}px | Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
        
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

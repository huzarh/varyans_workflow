#!/usr/bin/env python3
"""
Optimize edilmiş drone kontrol sistemi
"""
from core.state_manager import StateManager
from core.mavlink_manager import MAVLinkManager
from core.config import camera_config, detection_config, servo_config, mission_config, velocity_config, MAVLINK_CONNECTION

class DroneController:
    """Drone kontrol sınıfı"""
    
    def __init__(self):
        self.mavlink = MAVLinkManager(MAVLINK_CONNECTION)
    
    def guided_approach_velocity(self, state: StateManager) -> bool:
        """Hedef yönlendirme ve yük bırakma kontrolü"""
        print("🎯 Hedef yönlendirme kontrolü başlatılıyor...")
        
        # Görev tamamlandıysa RTL moduna geç
        if state.is_mission_completed():
            print("🎯 GÖREV TAMAMLANDI! RTL moduna geçiliyor...")
            return self._switch_to_rtl_mode()
        
        # MAVLink bağlantısı kur
        if not self.mavlink.connect():
            return False
        
        try:
            if state.state["target_detected"]:
                return self._handle_target_detected(state)
            else:
                print("❌ Hedef algılanmadı - hareket durduruluyor")
                self.mavlink.send_velocity_command(0, 0, 0)
                return True
        finally:
            self.mavlink.disconnect()
    
    def _handle_target_detected(self, state: StateManager) -> bool:
        """Hedef algılandığında işlem yap"""
        target = state.state["target_info"]
        cx = target.get("cx", 0)
        cy = target.get("cy", 0)
        bbox_area = target.get("bbox_area", 0)
        target_type = target.get("type", "unknown")
        
        # None değer kontrolü
        if bbox_area is None:
            bbox_area = 0
        if cx is None:
            cx = 0
        if cy is None:
            cy = 0
        
        # Merkez hesaplama
        center_x = camera_config.width // 2
        center_y = camera_config.height // 2
        offset_x = cx - center_x
        offset_y = cy - center_y
        total_offset = (offset_x**2 + offset_y**2)**0.5
        
        if total_offset <= detection_config.center_tolerance:
            return self._handle_cargo_drop(state, target_type, total_offset)
        else:
            return self._handle_direction_control(cx, cy, bbox_area, offset_x, offset_y, total_offset)
    
    def _handle_cargo_drop(self, state: StateManager, target_type: str, total_offset: float) -> bool:
        """Yük bırakma işlemi"""
        print(f"🎯 HEDEF MERKEZE GELDİ! Yük bırakma başlatılıyor...")
        print(f"📍 Hedef türü: {target_type}")
        print(f"�� Merkez offset: {total_offset:.1f} pixel (tolerans: {detection_config.center_tolerance})")
        
        # Yük bırakma
        if not self._drop_cargo(target_type):
            return False
        
        # Yük sayacını artır
        mission_completed = state.increment_cargo_dropped()
        
        # Hedefi temizle
        state.clear_target()
        print("🎯 Hedef temizlendi - yeni hedef aranacak")
        
        # Görev tamamlandıysa RTL moduna geç
        if mission_completed:
            print("🎯 TÜM YÜKLER BIRAKILDI! RTL moduna geçiliyor...")
            return self._switch_to_rtl_mode()
        
        # Drone'u durdur
        self.mavlink.send_velocity_command(0, 0, 0)
        print("🛑 Hedef merkeze geldi - drone durduruldu")
        return True
    
    def _drop_cargo(self, target_type: str) -> bool:
        """Yük bırakma işlemi"""
        try:
            if "blue" in target_type.lower():
                print("🔵 Mavi yük bırakılıyor...")
                pwm_value = servo_config.blue_cargo_pwm
            elif "red" in target_type.lower():
                print("🔴 Kırmızı yük bırakılıyor...")
                pwm_value = servo_config.red_cargo_pwm
            else:
                print(f"⚠️ Bilinmeyen hedef türü: {target_type}")
                return False
            
            # Servo komutunu gönder
            if not self.mavlink.send_servo_command(servo_config.servo_id, pwm_value):
                return False
            
            print(f"✅ Servo {servo_config.servo_id} {pwm_value} PWM'e ayarlandı")
            
            # Bekle
            import time
            time.sleep(servo_config.servo_wait_sec)
            
            # Servoyu normal pozisyona döndür
            if not self.mavlink.send_servo_command(servo_config.servo_id, servo_config.normal_pwm):
                return False
            
            print(f"✅ Servo normal pozisyona döndürüldü ({servo_config.normal_pwm} PWM)")
            return True
            
        except Exception as e:
            print(f"❌ Yük bırakma hatası: {e}")
            return False
    
    def _handle_direction_control(self, cx: int, cy: int, bbox_area: int, offset_x: int, offset_y: int, total_offset: float) -> bool:
        """Yön kontrolü işlemi"""
        center_x = camera_config.width // 2
        center_y = camera_config.height // 2
        
        # None değer kontrolü
        if bbox_area is None:
            bbox_area = 0
        if cx is None:
            cx = 0
        if cy is None:
            cy = 0
        
        # Yatay yön kontrolü (vy - sağ/sol hareket)
        if abs(offset_x) > 50:  # 50 pixel tolerans
            vy = -offset_x / center_x * velocity_config.direction_factor_x
            vy = max(-velocity_config.max_vy, min(velocity_config.max_vy, vy))
        else:
            vy = 0
        
        # Dikey yön kontrolü (vz - yukarı/aşağı hareket)
        if abs(offset_y) > 50:  # 50 pixel tolerans
            vz = offset_y / center_y * velocity_config.direction_factor_y
            vz = max(-velocity_config.max_vz, min(velocity_config.max_vz, vz))
        else:
            vz = 0
        
        # İleri hareket (vx) - hedefe yaklaşma
        # bbox_area None kontrolü eklendi
        if bbox_area is not None and bbox_area > 0:
            distance = max(0.5, 10.0 - (bbox_area / 1000.0))
        else:
            distance = 5.0  # Varsayılan mesafe
        
        vx = min(velocity_config.max_vx, distance * velocity_config.approach_factor)
        
        # Velocity komutunu gönder
        if not self.mavlink.send_velocity_command(vx, vy, vz):
            return False
        
        # Yön bilgilerini yazdır
        direction = self._get_direction_string(offset_x, offset_y)
        print(f"🎯 Hedef: {direction} | Offset: ({offset_x:.0f}, {offset_y:.0f}) | Merkez mesafesi: {total_offset:.1f}px | Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
        return True
    
    def _get_direction_string(self, offset_x: int, offset_y: int) -> str:
        """Yön string'ini oluştur"""
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
        return direction.strip()
    
    def _switch_to_rtl_mode(self) -> bool:
        """RTL moduna geçiş"""
        print("🏠 RTL moduna geçiş yapılıyor...")
        
        if not self.mavlink.connect():
            return False
        
        try:
            return self.mavlink.set_mode(mission_config.rtl_mode_id, "RTL")
        finally:
            self.mavlink.disconnect()

# Backward compatibility için eski fonksiyonlar
def guided_approach_velocity(state: StateManager) -> bool:
    """Backward compatibility için eski fonksiyon"""
    controller = DroneController()
    return controller.guided_approach_velocity(state)

def switch_to_rtl_mode() -> bool:
    """Backward compatibility için eski fonksiyon"""
    controller = DroneController()
    return controller._switch_to_rtl_mode()

#!/usr/bin/env python3
"""
Optimize edilmiş state yönetimi
"""
from multiprocessing import Manager
from typing import Dict, Any, Optional
from dataclasses import dataclass
from core.config import mission_config

@dataclass
class TargetInfo:
    """Hedef bilgileri için data class"""
    type: Optional[str] = None
    cx: Optional[int] = None
    cy: Optional[int] = None
    bbox_area: Optional[int] = None
    confidence: float = 0.0

class StateManager:
    """Optimize edilmiş state yöneticisi"""
    
    def __init__(self):
        mgr = Manager()
        self.state = mgr.dict()
        self._initialize_state()
    
    def _initialize_state(self):
        """State'i başlangıç değerleriyle initialize et"""
        self.state.update({
            "target_detected": False,
            "target_info": {
                "type": None,
                "cx": None,
                "cy": None,
                "bbox_area": None,
                "confidence": 0.0
            },
            "mission_completed": False,
            "cargo_dropped": 0,
            "max_cargo": mission_config.max_cargo
        })
    
    def update_target(self, target_type: str, cx: int, cy: int, bbox_area: int, confidence: float) -> None:
        """Hedef bilgilerini güncelle"""
        # None değer kontrolü ve varsayılan değerler
        if target_type is None:
            target_type = "unknown"
        if cx is None:
            cx = 0
        if cy is None:
            cy = 0
        if bbox_area is None:
            bbox_area = 0
        if confidence is None:
            confidence = 0.0
        
        print(f"----------> {target_type} {cx} {cy} {bbox_area} {confidence}")
        
        self.state["target_detected"] = True
        self.state["target_info"].update({
            "type": target_type,
            "cx": cx,
            "cy": cy,
            "bbox_area": bbox_area,
            "confidence": confidence,
        })
    
    def clear_target(self) -> None:
        """Hedef bilgilerini temizle"""
        self.state["target_detected"] = False
        # Target info'yu sıfırla
        self.state["target_info"].update({
            "type": None,
            "cx": None,
            "cy": None,
            "bbox_area": None,
            "confidence": 0.0
        })
    
    def increment_cargo_dropped(self) -> bool:
        """Yük bırakıldığında sayacı artır ve görev durumunu kontrol et"""
        self.state["cargo_dropped"] += 1
        current = self.state["cargo_dropped"]
        max_cargo = self.state["max_cargo"]
        
        print(f"📦 Yük bırakıldı! Toplam: {current}/{max_cargo}")
        
        # Maksimum yük sayısına ulaşıldı mı?
        if current >= max_cargo:
            self.state["mission_completed"] = True
            print("🎯 GÖREV TAMAMLANDI! Tüm yükler bırakıldı.")
            return True
        return False
    
    def is_mission_completed(self) -> bool:
        """Görev tamamlandı mı?"""
        return self.state["mission_completed"]
    
    def get_cargo_status(self) -> Dict[str, int]:
        """Yük durumunu döndür"""
        dropped = self.state["cargo_dropped"]
        max_cargo = self.state["max_cargo"]
        return {
            "dropped": dropped,
            "max": max_cargo,
            "remaining": max_cargo - dropped
        }
    
    def reset_mission(self) -> None:
        """Görevi sıfırla"""
        self.state["mission_completed"] = False
        self.state["cargo_dropped"] = 0
        print("🔄 Görev sıfırlandı - yeni görev başlatılıyor")
    
    def get_target_info(self) -> Optional[Dict[str, Any]]:
        """Hedef bilgilerini döndür"""
        if self.state["target_detected"]:
            return dict(self.state["target_info"])
        return None
    
    def is_target_detected(self) -> bool:
        """Hedef algılandı mı?"""
        return self.state["target_detected"]

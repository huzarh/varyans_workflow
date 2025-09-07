# core/state_manager.py
from multiprocessing import Manager

class StateManager:
    def __init__(self):
        mgr = Manager()
        self.state = mgr.dict()
        # initial vars
        self.state["target_detected"] = False
        self.state["target_info"] = {
            "type":None,
            "cx": None,
            "cy": None,
            "bbox_area": None,
            "confidence": 0.0
        }
        # Görev durumu ve yük sayacı
        self.state["mission_completed"] = False
        self.state["cargo_dropped"] = 0
        self.state["max_cargo"] = 2  # Maksimum 2 yük bırakılacak

    def update_target(self,type, cx, cy, bbox_area, confidence):
        # test print
        print("----------> " + type, cx, cy, bbox_area, confidence)
        self.state["target_detected"] = True
        self.state["target_info"] = {
            "type":type, 
            "cx": cx,
            "cy": cy,
            "bbox_area": bbox_area,
            "confidence": confidence,
        }

    def clear_target(self):
        self.state["target_detected"] = False

    def increment_cargo_dropped(self):
        """Yük bırakıldığında sayacı artır"""
        self.state["cargo_dropped"] += 1
        print(f"📦 Yük bırakıldı! Toplam: {self.state['cargo_dropped']}/{self.state['max_cargo']}")
        
        # Maksimum yük sayısına ulaşıldı mı?
        if self.state["cargo_dropped"] >= self.state["max_cargo"]:
            self.state["mission_completed"] = True
            print("🎯 GÖREV TAMAMLANDI! Tüm yükler bırakıldı.")
            return True
        return False

    def is_mission_completed(self):
        """Görev tamamlandı mı?"""
        return self.state["mission_completed"]

    def get_cargo_status(self):
        """Yük durumunu döndür"""
        return {
            "dropped": self.state["cargo_dropped"],
            "max": self.state["max_cargo"],
            "remaining": self.state["max_cargo"] - self.state["cargo_dropped"]
        }

    def reset_mission(self):
        """Görevi sıfırla"""
        self.state["mission_completed"] = False
        self.state["cargo_dropped"] = 0
        print("🔄 Görev sıfırlandı - yeni görev başlatılıyor")

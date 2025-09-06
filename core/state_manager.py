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

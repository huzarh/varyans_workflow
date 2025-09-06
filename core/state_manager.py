# core/state_manager.py
from multiprocessing import Manager

class StateManager:
    def __init__(self):
        mgr = Manager()
        self.state = mgr.dict()
        # initial vars
        self.state["target_detected"] = False
        self.state["target_info"] = {
            "cx": None,
            "cy": None,
            "bearing": None,
            "distance": None,
            "confidence": 0.0
        }

    def update_target(self, cx, cy, bearing, distance, confidence):
        self.state["target_detected"] = True
        self.state["target_info"] = {
            "cx": cx,
            "cy": cy,
            "bearing": bearing,
            "distance": distance,
            "confidence": confidence,
        }

    def clear_target(self):
        self.state["target_detected"] = False

"""
State Management System for Raspberry Pi 5 Autonomous Flight
Uses multiprocessing.Manager.dict() for thread-safe inter-process communication
"""

import multiprocessing
import logging
import time
from typing import Dict, Any, Optional
from threading import Lock
from dataclasses import dataclass
from datetime import datetime

@dataclass
class ServoState:
    """Servo state data structure"""
    id: int
    pwm: int
    angle: float
    timestamp: float
    status: str = "idle"  # idle, moving, error

@dataclass
class FlightState:
    """Flight state data structure"""
    armed: bool = False
    mode: str = "STABILIZE"
    altitude: float = 0.0
    heading: float = 0.0
    battery_voltage: float = 0.0
    timestamp: float = 0.0

class StateManager:
    """
    Centralized state management using multiprocessing.Manager.dict()
    Provides thread-safe access to shared state across all processes
    """
    
    def __init__(self):
        # Create multiprocessing manager for shared state
        self.manager = multiprocessing.Manager()
        self.shared_state = self.manager.dict()
        
        # Thread locks for critical sections
        self._servo_lock = Lock()
        self._flight_lock = Lock()
        
        # Initialize default state
        self._initialize_default_state()
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
    
    def _initialize_default_state(self):
        """Initialize default system state"""
        current_time = time.time()
        
        # Servo states
        self.shared_state['servo_5'] = {
            'id': 5,
            'pwm': 1800,
            'angle': 0.0,
            'timestamp': current_time,
            'status': 'idle'
        }
        
        self.shared_state['servo_7'] = {
            'id': 7,
            'pwm': 1800,
            'angle': 0.0,
            'timestamp': current_time,
            'status': 'idle'
        }
        
        # Flight state
        self.shared_state['flight'] = {
            'armed': False,
            'mode': 'STABILIZE',
            'altitude': 0.0,
            'heading': 0.0,
            'battery_voltage': 0.0,
            'timestamp': current_time
        }
        
        # System state
        self.shared_state['system'] = {
            'status': 'initializing',
            'uptime': 0.0,
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'timestamp': current_time
        }
        
        # Vision state (placeholder for future implementation)
        self.shared_state['vision'] = {
            'target_detected': False,
            'target_x': 0.0,
            'target_y': 0.0,
            'confidence': 0.0,
            'timestamp': current_time
        }
    
    def get_servo_state(self, servo_id: int) -> Optional[Dict[str, Any]]:
        """Get current servo state"""
        with self._servo_lock:
            key = f'servo_{servo_id}'
            return self.shared_state.get(key)
    
    def update_servo_state(self, servo_id: int, pwm: int, angle: float = None, status: str = "moving"):
        """Update servo state with thread safety"""
        with self._servo_lock:
            key = f'servo_{servo_id}'
            if key in self.shared_state:
                self.shared_state[key].update({
                    'pwm': pwm,
                    'angle': angle or self._pwm_to_angle(pwm),
                    'timestamp': time.time(),
                    'status': status
                })
                self.logger.debug(f"Updated servo {servo_id}: PWM={pwm}, Angle={angle}")
    
    def get_flight_state(self) -> Dict[str, Any]:
        """Get current flight state"""
        with self._flight_lock:
            return self.shared_state.get('flight', {})
    
    def update_flight_state(self, **kwargs):
        """Update flight state with thread safety"""
        with self._flight_lock:
            if 'flight' in self.shared_state:
                self.shared_state['flight'].update(kwargs)
                self.shared_state['flight']['timestamp'] = time.time()
    
    def get_vision_state(self) -> Dict[str, Any]:
        """Get current vision state"""
        return self.shared_state.get('vision', {})
    
    def update_vision_state(self, **kwargs):
        """Update vision state"""
        if 'vision' in self.shared_state:
            self.shared_state['vision'].update(kwargs)
            self.shared_state['vision']['timestamp'] = time.time()
    
    def get_system_state(self) -> Dict[str, Any]:
        """Get current system state"""
        return self.shared_state.get('system', {})
    
    def update_system_state(self, **kwargs):
        """Update system state"""
        if 'system' in self.shared_state:
            self.shared_state['system'].update(kwargs)
            self.shared_state['system']['timestamp'] = time.time()
    
    def _pwm_to_angle(self, pwm: int) -> float:
        """Convert PWM value to angle (simplified conversion)"""
        # Assuming 1000-2000 PWM range maps to -90 to +90 degrees
        return ((pwm - 1500) / 500) * 90.0
    
    def get_all_state(self) -> Dict[str, Any]:
        """Get complete system state (for debugging/monitoring)"""
        return dict(self.shared_state)
    
    def reset_servo_states(self):
        """Reset all servo states to default"""
        with self._servo_lock:
            for servo_id in [5, 7]:
                key = f'servo_{servo_id}'
                if key in self.shared_state:
                    self.shared_state[key].update({
                        'pwm': 1800,
                        'angle': 0.0,
                        'timestamp': time.time(),
                        'status': 'idle'
                    })
    
    def cleanup(self):
        """Cleanup resources"""
        self.manager.shutdown()

# Global state manager instance
state_manager = StateManager()

"""
Servo Control Module for Raspberry Pi 5
Handles PWM control of servos using pigpio library
"""

import pigpio
import time
import logging
from typing import Dict, Any, Optional
from config.settings import config
from core.state_manager import state_manager

class ServoController:
    """
    Servo control using pigpio for precise PWM generation
    Optimized for Raspberry Pi 5 with non-blocking operation
    """
    
    def __init__(self):
        self.pi = None
        self.logger = logging.getLogger(__name__)
        self.servo_config = config.get_servo_pwm_config()
        self.initialized = False
        
        # Servo state tracking
        self.servo_states = {}
        
    def initialize(self) -> bool:
        """Initialize pigpio and servo connections"""
        try:
            # Initialize pigpio daemon connection
            self.pi = pigpio.pi()
            
            if not self.pi.connected:
                self.logger.error("Failed to connect to pigpio daemon")
                return False
            
            # Initialize servos
            for servo_name, servo_config in self.servo_config.items():
                gpio_pin = servo_config['gpio_pin']
                pwm_value = servo_config['pwm']
                
                # Set PWM frequency (50Hz for servos)
                self.pi.set_PWM_frequency(gpio_pin, 50)
                
                # Set initial PWM value
                self.pi.set_servo_pulsewidth(gpio_pin, pwm_value)
                
                # Track servo state
                self.servo_states[servo_name] = {
                    'gpio_pin': gpio_pin,
                    'current_pwm': pwm_value,
                    'target_pwm': pwm_value
                }
                
                # Update state manager
                state_manager.update_servo_state(
                    servo_config['id'], 
                    pwm_value, 
                    status="initialized"
                )
                
                self.logger.info(f"Initialized {servo_name} on GPIO {gpio_pin} with PWM {pwm_value}")
            
            self.initialized = True
            self.logger.info("Servo controller initialized successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize servo controller: {e}")
            return False
    
    def set_servo_pwm(self, servo_id: int, pwm: int) -> bool:
        """Set servo PWM value with bounds checking"""
        try:
            # Find servo configuration
            servo_name = f'servo_{servo_id}'
            if servo_name not in self.servo_config:
                self.logger.error(f"Unknown servo ID: {servo_id}")
                return False
            
            servo_config = self.servo_config[servo_name]
            gpio_pin = servo_config['gpio_pin']
            
            # Bounds checking
            if not (servo_config['min_pwm'] <= pwm <= servo_config['max_pwm']):
                self.logger.warning(f"PWM {pwm} out of bounds for servo {servo_id}")
                pwm = max(servo_config['min_pwm'], min(pwm, servo_config['max_pwm']))
            
            # Set PWM value
            self.pi.set_servo_pulsewidth(gpio_pin, pwm)
            
            # Update internal state
            self.servo_states[servo_name]['current_pwm'] = pwm
            self.servo_states[servo_name]['target_pwm'] = pwm
            
            # Update state manager
            angle = self._pwm_to_angle(pwm)
            state_manager.update_servo_state(servo_id, pwm, angle, "moving")
            
            self.logger.debug(f"Set servo {servo_id} PWM to {pwm}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set servo {servo_id} PWM: {e}")
            return False
    
    def set_servo_angle(self, servo_id: int, angle: float) -> bool:
        """Set servo angle (converts to PWM internally)"""
        try:
            # Find servo configuration
            servo_name = f'servo_{servo_id}'
            if servo_name not in self.servo_config:
                self.logger.error(f"Unknown servo ID: {servo_id}")
                return False
            
            servo_config = self.servo_config[servo_name]
            
            # Convert angle to PWM (assuming -90 to +90 degrees maps to 1000-2000 PWM)
            pwm = int(1500 + (angle / 90.0) * 500)
            
            # Bounds checking
            pwm = max(servo_config['min_pwm'], min(pwm, servo_config['max_pwm']))
            
            return self.set_servo_pwm(servo_id, pwm)
            
        except Exception as e:
            self.logger.error(f"Failed to set servo {servo_id} angle: {e}")
            return False
    
    def get_servo_state(self, servo_id: int) -> Optional[Dict[str, Any]]:
        """Get current servo state"""
        servo_name = f'servo_{servo_id}'
        if servo_name in self.servo_states:
            return self.servo_states[servo_name].copy()
        return None
    
    def _pwm_to_angle(self, pwm: int) -> float:
        """Convert PWM value to angle"""
        return ((pwm - 1500) / 500) * 90.0
    
    def emergency_stop(self):
        """Emergency stop - set all servos to neutral position"""
        try:
            for servo_name, servo_config in self.servo_config.items():
                servo_id = servo_config['id']
                neutral_pwm = config.system.emergency_stop_pwm
                self.set_servo_pwm(servo_id, neutral_pwm)
            
            self.logger.warning("Emergency stop activated - all servos set to neutral")
            
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
    
    def test_servo_movement(self, servo_id: int, duration: float = 5.0):
        """Test servo movement pattern"""
        try:
            servo_name = f'servo_{servo_id}'
            if servo_name not in self.servo_config:
                self.logger.error(f"Unknown servo ID: {servo_id}")
                return
            
            servo_config = self.servo_config[servo_name]
            start_time = time.time()
            
            self.logger.info(f"Starting servo {servo_id} test for {duration} seconds")
            
            while time.time() - start_time < duration:
                # Sweep from min to max PWM
                for pwm in range(servo_config['min_pwm'], servo_config['max_pwm'] + 1, 50):
                    if time.time() - start_time >= duration:
                        break
                    
                    self.set_servo_pwm(servo_id, pwm)
                    time.sleep(0.1)
                
                # Return to center
                self.set_servo_pwm(servo_id, 1500)
                time.sleep(0.5)
            
            # Set to test initialization value
            self.set_servo_pwm(servo_id, servo_config['pwm'])
            self.logger.info(f"Servo {servo_id} test completed")
            
        except Exception as e:
            self.logger.error(f"Servo test failed: {e}")
    
    def cleanup(self):
        """Cleanup resources"""
        try:
            if self.pi and self.pi.connected:
                # Set all servos to neutral
                self.emergency_stop()
                
                # Stop PWM on all servo pins
                for servo_config in self.servo_config.values():
                    self.pi.set_servo_pulsewidth(servo_config['gpio_pin'], 0)
                
                self.pi.stop()
                self.logger.info("Servo controller cleaned up")
                
        except Exception as e:
            self.logger.error(f"Cleanup failed: {e}")

# Global servo controller instance
servo_controller = ServoController()

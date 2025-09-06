"""
Main Entry Point for Raspberry Pi 5 Autonomous Flight System
Phase 1: Servo initialization and test infrastructure
"""

import sys
import time
import signal
import logging
from multiprocessing import Process, Queue
from typing import Dict, Any

# Import core modules
from config.settings import config
from core.logger import setup_logging, get_logger, log_system_info, log_servo_test_results
from core.state_manager import state_manager
from core.servo_controller import servo_controller

# Import communication modules
from comms.udp_handler import udp_handler
from flight.mavlink_comm import mavlink_comm

# Import vision modules (placeholder)
from vision.camera_controller import camera_controller

class FlightSystem:
    """
    Main flight system controller
    Manages all subsystems and coordinates their operation
    """
    
    def __init__(self):
        self.logger = get_logger(__name__)
        self.running = False
        self.processes = {}
        
        # System state
        self.system_status = "initializing"
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def initialize(self) -> bool:
        """Initialize all system components"""
        try:
            self.logger.info("Initializing Raspberry Pi 5 Autonomous Flight System")
            
            # Log system information
            log_system_info()
            
            # Save MAVLink router configuration
            config.save_mavlink_config()
            
            # Initialize servo controller
            if not servo_controller.initialize():
                self.logger.error("Failed to initialize servo controller")
                return False
            
            # Initialize UDP communication
            if not udp_handler.initialize_local_udp():
                self.logger.error("Failed to initialize local UDP")
                return False
            
            if not udp_handler.initialize_mission_planner_udp():
                self.logger.warning("Failed to initialize Mission Planner UDP (non-critical)")
            
            # Initialize MAVLink communication
            if not mavlink_comm.initialize_udp():
                self.logger.warning("Failed to initialize MAVLink UDP (non-critical)")
            
            # Initialize camera (optional for Phase 1)
            if not camera_controller.initialize():
                self.logger.warning("Camera initialization failed (optional for Phase 1)")
            
            # Start UDP receive loop
            udp_handler.start_receive_loop()
            
            self.system_status = "initialized"
            self.logger.info("System initialization completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"System initialization failed: {e}")
            return False
    
    def run_servo_test(self, duration: float = 10.0):
        """Run comprehensive servo test"""
        try:
            self.logger.info("=" * 60)
            self.logger.info("STARTING SERVO TEST PHASE")
            self.logger.info("=" * 60)
            
            # Test servo 5
            self.logger.info("Testing Servo 5...")
            servo_controller.test_servo_movement(5, duration / 2)
            log_servo_test_results(5, True, "Movement test completed")
            
            # Test servo 7
            self.logger.info("Testing Servo 7...")
            servo_controller.test_servo_movement(7, duration / 2)
            log_servo_test_results(7, True, "Movement test completed")
            
            # Set servos to initialization position
            servo_controller.set_servo_pwm(5, 1800)
            servo_controller.set_servo_pwm(7, 1800)
            
            self.logger.info("=" * 60)
            self.logger.info("SERVO TEST PHASE COMPLETED")
            self.logger.info("=" * 60)
            
        except Exception as e:
            self.logger.error(f"Servo test failed: {e}")
            log_servo_test_results(0, False, str(e))
    
    def run_state_monitor(self, duration: float = 30.0):
        """Run state monitoring and display"""
        try:
            self.logger.info("Starting state monitoring...")
            start_time = time.time()
            
            while time.time() - start_time < duration:
                # Get current state
                servo_5_state = state_manager.get_servo_state(5)
                servo_7_state = state_manager.get_servo_state(7)
                flight_state = state_manager.get_flight_state()
                system_state = state_manager.get_system_state()
                
                # Log state information
                self.logger.info(f"Servo 5: PWM={servo_5_state['pwm']}, Status={servo_5_state['status']}")
                self.logger.info(f"Servo 7: PWM={servo_7_state['pwm']}, Status={servo_7_state['status']}")
                self.logger.info(f"Flight: Armed={flight_state['armed']}, Mode={flight_state['mode']}")
                self.logger.info(f"System: Status={system_state['status']}")
                
                time.sleep(2.0)  # Update every 2 seconds
            
            self.logger.info("State monitoring completed")
            
        except Exception as e:
            self.logger.error(f"State monitoring failed: {e}")
    
    def run(self):
        """Main system run loop"""
        try:
            self.running = True
            self.system_status = "running"
            
            self.logger.info("Flight system started - Phase 1: Servo Testing")
            
            # Phase 1: Servo initialization and testing
            self.run_servo_test(duration=10.0)
            
            # State monitoring
            self.run_state_monitor(duration=30.0)
            
            # Keep system running for additional testing
            self.logger.info("System running - Press Ctrl+C to exit")
            while self.running:
                time.sleep(1.0)
                
                # Update system state
                state_manager.update_system_state(
                    status="running",
                    uptime=time.time(),
                    cpu_usage=0.0,  # Placeholder
                    memory_usage=0.0  # Placeholder
                )
            
        except KeyboardInterrupt:
            self.logger.info("Received interrupt signal")
        except Exception as e:
            self.logger.error(f"System run error: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Graceful system shutdown"""
        try:
            self.logger.info("Initiating system shutdown...")
            self.running = False
            self.system_status = "shutting_down"
            
            # Stop UDP receive loop
            udp_handler.stop_receive_loop()
            
            # Emergency stop servos
            servo_controller.emergency_stop()
            
            # Cleanup all components
            servo_controller.cleanup()
            udp_handler.cleanup()
            mavlink_comm.cleanup()
            camera_controller.cleanup()
            state_manager.cleanup()
            
            self.system_status = "stopped"
            self.logger.info("System shutdown completed")
            
        except Exception as e:
            self.logger.error(f"Shutdown error: {e}")
    
    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        self.logger.info(f"Received signal {signum}")
        self.running = False

def main():
    """Main entry point"""
    try:
        # Create and initialize flight system
        flight_system = FlightSystem()
        
        if not flight_system.initialize():
            print("System initialization failed - check logs")
            sys.exit(1)
        
        # Run the system
        flight_system.run()
        
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

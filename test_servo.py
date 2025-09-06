"""
Standalone Servo Test Script
Quick test for servo initialization and movement
"""

import sys
import time
import logging
from core.servo_controller import servo_controller
from core.logger import setup_logging, get_logger

def test_servo_initialization():
    """Test servo initialization and basic movement"""
    logger = get_logger(__name__)
    
    try:
        logger.info("Starting servo initialization test...")
        
        # Initialize servo controller
        if not servo_controller.initialize():
            logger.error("Failed to initialize servo controller")
            return False
        
        logger.info("Servo controller initialized successfully")
        
        # Test servo 5
        logger.info("Testing servo 5...")
        servo_controller.set_servo_pwm(5, 1800)
        time.sleep(1)
        servo_controller.set_servo_pwm(5, 1500)
        time.sleep(1)
        servo_controller.set_servo_pwm(5, 1800)
        
        # Test servo 7
        logger.info("Testing servo 7...")
        servo_controller.set_servo_pwm(7, 1800)
        time.sleep(1)
        servo_controller.set_servo_pwm(7, 1500)
        time.sleep(1)
        servo_controller.set_servo_pwm(7, 1800)
        
        logger.info("Servo test completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Servo test failed: {e}")
        return False
    
    finally:
        # Cleanup
        servo_controller.cleanup()

def main():
    """Main test function"""
    setup_logging()
    logger = get_logger(__name__)
    
    logger.info("=" * 50)
    logger.info("SERVO INITIALIZATION TEST")
    logger.info("=" * 50)
    
    success = test_servo_initialization()
    
    if success:
        logger.info("✅ Servo test PASSED")
        sys.exit(0)
    else:
        logger.error("❌ Servo test FAILED")
        sys.exit(1)

if __name__ == "__main__":
    main()

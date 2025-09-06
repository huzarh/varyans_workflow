"""
Logging Configuration for Raspberry Pi 5 Autonomous Flight System
Optimized for Pi 5 with efficient logging and rotation
"""

import logging
import logging.handlers
import os
from datetime import datetime
from config.settings import config

def setup_logging():
    """Setup comprehensive logging system for the flight system"""
    
    # Create logs directory if it doesn't exist
    os.makedirs('logs', exist_ok=True)
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, config.system.log_level))
    
    # Clear existing handlers
    root_logger.handlers.clear()
    
    # Create formatters
    detailed_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    simple_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Console handler (simple format for real-time monitoring)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(simple_formatter)
    
    # File handler with rotation (detailed format for debugging)
    file_handler = logging.handlers.RotatingFileHandler(
        config.system.log_file,
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(detailed_formatter)
    
    # Error file handler (errors only)
    error_handler = logging.handlers.RotatingFileHandler(
        'logs/errors.log',
        maxBytes=5*1024*1024,  # 5MB
        backupCount=3
    )
    error_handler.setLevel(logging.ERROR)
    error_handler.setFormatter(detailed_formatter)
    
    # Add handlers to root logger
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(error_handler)
    
    # Configure specific loggers
    _configure_module_loggers()
    
    # Log system startup
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Raspberry Pi 5 Autonomous Flight System Starting")
    logger.info(f"Log level: {config.system.log_level}")
    logger.info(f"Log file: {config.system.log_file}")
    logger.info("=" * 60)

def _configure_module_loggers():
    """Configure specific module loggers with appropriate levels"""
    
    # Servo controller - verbose for debugging
    servo_logger = logging.getLogger('core.servo_controller')
    servo_logger.setLevel(logging.DEBUG)
    
    # State manager - info level
    state_logger = logging.getLogger('core.state_manager')
    state_logger.setLevel(logging.INFO)
    
    # MAVLink communication - info level
    mavlink_logger = logging.getLogger('flight.mavlink_comm')
    mavlink_logger.setLevel(logging.INFO)
    
    # Vision processing - debug level (will be verbose during development)
    vision_logger = logging.getLogger('vision')
    vision_logger.setLevel(logging.DEBUG)
    
    # UDP communication - info level
    udp_logger = logging.getLogger('comms.udp_handler')
    udp_logger.setLevel(logging.INFO)

def get_logger(name: str) -> logging.Logger:
    """Get a logger instance for a specific module"""
    return logging.getLogger(name)

def log_system_info():
    """Log system information for debugging"""
    logger = logging.getLogger(__name__)
    
    try:
        import platform
        import psutil
        
        logger.info(f"Python version: {platform.python_version()}")
        logger.info(f"Platform: {platform.platform()}")
        logger.info(f"CPU count: {psutil.cpu_count()}")
        logger.info(f"Memory total: {psutil.virtual_memory().total / (1024**3):.1f} GB")
        logger.info(f"Memory available: {psutil.virtual_memory().available / (1024**3):.1f} GB")
        
    except ImportError:
        logger.warning("psutil not available - system info logging skipped")

def log_servo_test_results(servo_id: int, success: bool, details: str = ""):
    """Log servo test results in a structured format"""
    logger = logging.getLogger('servo_test')
    
    status = "PASS" if success else "FAIL"
    logger.info(f"SERVO_TEST - ID:{servo_id} - {status} - {details}")

def log_flight_event(event_type: str, details: str = ""):
    """Log flight events in a structured format"""
    logger = logging.getLogger('flight_events')
    
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    logger.info(f"FLIGHT_EVENT - {timestamp} - {event_type} - {details}")

def log_vision_event(event_type: str, details: str = ""):
    """Log vision processing events"""
    logger = logging.getLogger('vision_events')
    
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    logger.info(f"VISION_EVENT - {timestamp} - {event_type} - {details}")

# Initialize logging when module is imported
setup_logging()

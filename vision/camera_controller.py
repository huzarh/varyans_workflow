"""
Camera Controller for Raspberry Pi 5 with Pi Camera v3
Handles camera initialization and basic image capture
"""

import logging
from typing import Optional, Tuple
from config.settings import config

class CameraController:
    """
    Pi Camera v3 controller for target detection
    Optimized for Raspberry Pi 5 performance
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.camera = None
        self.initialized = False
        
        # Camera configuration
        self.resolution = config.camera.resolution
        self.framerate = config.camera.framerate
        self.rotation = config.camera.rotation
    
    def initialize(self) -> bool:
        """Initialize Pi Camera v3"""
        try:
            # Import picamera2 when available
            try:
                from picamera2 import Picamera2
                from libcamera import controls
            except ImportError:
                self.logger.error("picamera2 not available - install with: pip install picamera2")
                return False
            
            # Initialize camera
            self.camera = Picamera2()
            
            # Configure camera
            camera_config = self.camera.create_still_configuration(
                main={"size": self.resolution, "format": "RGB888"},
                lores={"size": (640, 480), "format": "YUV420"}
            )
            
            self.camera.configure(camera_config)
            self.camera.start()
            
            # Set camera controls
            self.camera.set_controls({
                "ExposureTime": 10000,  # 10ms exposure
                "AnalogueGain": 1.0,
                "DigitalGain": 1.0
            })
            
            self.initialized = True
            self.logger.info(f"Camera initialized: {self.resolution} @ {self.framerate}fps")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize camera: {e}")
            return False
    
    def capture_image(self) -> Optional[bytes]:
        """Capture a single image"""
        try:
            if not self.initialized or not self.camera:
                self.logger.error("Camera not initialized")
                return None
            
            # Capture image
            image = self.camera.capture_array()
            
            self.logger.debug(f"Captured image: {image.shape}")
            return image
            
        except Exception as e:
            self.logger.error(f"Failed to capture image: {e}")
            return None
    
    def get_camera_info(self) -> dict:
        """Get camera information"""
        if not self.initialized:
            return {"status": "not_initialized"}
        
        return {
            "status": "initialized",
            "resolution": self.resolution,
            "framerate": self.framerate,
            "rotation": self.rotation
        }
    
    def cleanup(self):
        """Cleanup camera resources"""
        try:
            if self.camera:
                self.camera.stop()
                self.camera.close()
                self.logger.info("Camera cleaned up")
                
        except Exception as e:
            self.logger.error(f"Camera cleanup failed: {e}")

# Global camera controller instance
camera_controller = CameraController()

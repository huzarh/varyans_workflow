"""
Central Configuration Module for Raspberry Pi 5 Autonomous Flight System
Manages all system settings, MAVLink routing, UDP endpoints, and hardware configurations
"""

import os
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class MAVLinkConfig:
    """MAVLink Router Configuration for Pi 5"""
    # General settings optimized for Pi 5
    tcp_server_port: int = 5760
    report_stats: bool = False  # Reduces CPU load on Pi 5
    
    # Pixhawk USB connection - Primary flight control channel
    pixhawk_device: str = "/dev/ttyACM0"
    pixhawk_baud: int = 57600
    
    # Mission Planner UDP - Monitoring and waypoint assignment only
    mission_planner_ip: str = "10.7.225.60"
    mission_planner_port: int = 14550
    
    # Local UDP - Inter-script communication on Pi
    local_udp_ip: str = "127.0.0.1"
    local_udp_port: int = 14540

@dataclass
class ServoConfig:
    """Servo control configuration"""
    # Servo IDs for payload control
    servo_5_id: int = 5
    servo_7_id: int = 7
    
    # PWM values (1000-2000 microseconds)
    default_pwm: int = 1800  # Test initialization value
    min_pwm: int = 1000
    max_pwm: int = 2000
    
    # Pi GPIO settings
    gpio_pin_5: int = 18  # Physical pin 12
    gpio_pin_7: int = 19  # Physical pin 35

@dataclass
class CameraConfig:
    """Pi Camera v3 configuration"""
    resolution: tuple = (1920, 1080)
    framerate: int = 30
    rotation: int = 0
    # Vision processing will be added in later phases

@dataclass
class SystemConfig:
    """System-wide configuration"""
    # Logging
    log_level: str = "INFO"
    log_file: str = "logs/flight_system.log"
    
    # State management
    state_update_interval: float = 0.1  # 10Hz state updates
    
    # Pi 5 optimizations
    cpu_governor: str = "performance"  # For real-time performance
    memory_limit_mb: int = 512  # Reserve memory for vision processing
    
    # Safety settings
    max_servo_angle: float = 45.0  # degrees
    emergency_stop_pwm: int = 1500  # Neutral position

class ConfigManager:
    """Centralized configuration management"""
    
    def __init__(self):
        self.mavlink = MAVLinkConfig()
        self.servo = ServoConfig()
        self.camera = CameraConfig()
        self.system = SystemConfig()
        
        # Ensure data directories exist
        self._ensure_directories()
    
    def _ensure_directories(self):
        """Create necessary directories if they don't exist"""
        directories = ["logs", "cache", "data"]
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
    
    def get_mavlink_router_config(self) -> str:
        """Generate MAVLink router configuration string"""
        config = f"""[General]
TcpServerPort={self.mavlink.tcp_server_port}
ReportStats={str(self.mavlink.report_stats).lower()}
# TcpServerPort: Optional, for telemetry or simulation connections
# ReportStats: false → Reduces CPU load on Pi 5

[UartEndpoint pixhawk]
Device={self.mavlink.pixhawk_device}
Baud={self.mavlink.pixhawk_baud}
# USB connection to Pixhawk
# This endpoint is the primary MAVLink channel between Pi and flight controller
# All commands and data (position, attitude, RC override, etc.) pass through here
# Consider Pi 5 CPU and USB bandwidth during vision processing

[UdpEndpoint missionplanner]
Mode=Normal
Address={self.mavlink.mission_planner_ip}
Port={self.mavlink.mission_planner_port}
# Mission Planner on PC
# Pi -> PC UDP stream for waypoint assignment and telemetry (optional)
# Mission Planner IP must be static, port 14550 is standard MAVLink UDP port
# For monitoring/waypoint assignment only, critical flight commands don't go through here

[UdpEndpoint localudp]
Mode=Normal
Address={self.mavlink.local_udp_ip}
Port={self.mavlink.local_udp_port}
# MAVLink UDP channel for Pi scripts
# Vision, flight logic, or other modules communicate with Pixhawk through here
# Example: Vision script calculates target position -> sends to flight script via UDP
# Ports must not conflict, keep separate from Mission Planner

# Rules / Recommendations:
# 1️⃣ Pixhawk USB: all flight commands and critical data here
# 2️⃣ Mission Planner UDP: monitoring and waypoint assignment, not critical for flight control
# 3️⃣ Local UDP: inter-module communication within Pi
# 4️⃣ Fail-safe: if Mission Planner disconnects, flight continues smoothly via Pixhawk
# 5️⃣ Pi 5 resource management: monitor UDP and USB bandwidth, especially during vision processing
# 6️⃣ During flight, ports and IPs must remain static, no DHCP changes
# 7️⃣ Use current Raspbian/Pi OS with MAVLink Router and libmavlink compatible versions
"""
        return config
    
    def get_servo_pwm_config(self) -> Dict[str, Any]:
        """Get servo PWM configuration for initialization"""
        return {
            "servo_5": {
                "id": self.servo.servo_5_id,
                "gpio_pin": self.servo.gpio_pin_5,
                "pwm": self.servo.default_pwm,
                "min_pwm": self.servo.min_pwm,
                "max_pwm": self.servo.max_pwm
            },
            "servo_7": {
                "id": self.servo.servo_7_id,
                "gpio_pin": self.servo.gpio_pin_7,
                "pwm": self.servo.default_pwm,
                "min_pwm": self.servo.min_pwm,
                "max_pwm": self.servo.max_pwm
            }
        }
    
    def save_mavlink_config(self, filepath: str = "mavlink_router.conf"):
        """Save MAVLink router configuration to file"""
        with open(filepath, 'w') as f:
            f.write(self.get_mavlink_router_config())
        print(f"MAVLink router configuration saved to {filepath}")

# Global configuration instance
config = ConfigManager()

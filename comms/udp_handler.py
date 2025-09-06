"""
UDP Communication Handler for Raspberry Pi 5
Manages UDP communication between local scripts and Mission Planner
"""

import socket
import threading
import time
import logging
from typing import Optional, Callable, Dict, Any
from config.settings import config

class UDPHandler:
    """
    UDP communication handler for inter-script communication
    Manages both local UDP and Mission Planner UDP channels
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # UDP sockets
        self.local_socket = None
        self.mission_planner_socket = None
        
        # Communication state
        self.local_connected = False
        self.mission_planner_connected = False
        
        # Message handlers
        self.message_handlers = {}
        
        # Threading
        self.receive_thread = None
        self.running = False
    
    def initialize_local_udp(self) -> bool:
        """Initialize local UDP socket for inter-script communication"""
        try:
            self.local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.local_socket.bind((config.mavlink.local_udp_ip, config.mavlink.local_udp_port))
            self.local_socket.settimeout(1.0)
            
            self.local_connected = True
            self.logger.info(f"Local UDP initialized: {config.mavlink.local_udp_ip}:{config.mavlink.local_udp_port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize local UDP: {e}")
            return False
    
    def initialize_mission_planner_udp(self) -> bool:
        """Initialize Mission Planner UDP socket"""
        try:
            self.mission_planner_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mission_planner_socket.settimeout(1.0)
            
            self.mission_planner_connected = True
            self.logger.info(f"Mission Planner UDP initialized: {config.mavlink.mission_planner_ip}:{config.mavlink.mission_planner_port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize Mission Planner UDP: {e}")
            return False
    
    def send_local_message(self, message: Dict[str, Any]) -> bool:
        """Send message to local UDP channel"""
        try:
            if not self.local_connected or not self.local_socket:
                self.logger.error("Local UDP not initialized")
                return False
            
            # Convert message to bytes (placeholder for actual serialization)
            message_bytes = str(message).encode('utf-8')
            
            # Send to local UDP
            self.local_socket.sendto(message_bytes, (config.mavlink.local_udp_ip, config.mavlink.local_udp_port))
            
            self.logger.debug(f"Local message sent: {message}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send local message: {e}")
            return False
    
    def send_mission_planner_message(self, message: Dict[str, Any]) -> bool:
        """Send message to Mission Planner"""
        try:
            if not self.mission_planner_connected or not self.mission_planner_socket:
                self.logger.error("Mission Planner UDP not initialized")
                return False
            
            # Convert message to bytes (placeholder for MAVLink serialization)
            message_bytes = str(message).encode('utf-8')
            
            # Send to Mission Planner
            self.mission_planner_socket.sendto(
                message_bytes, 
                (config.mavlink.mission_planner_ip, config.mavlink.mission_planner_port)
            )
            
            self.logger.debug(f"Mission Planner message sent: {message}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send Mission Planner message: {e}")
            return False
    
    def register_message_handler(self, message_type: str, handler: Callable):
        """Register a message handler for specific message types"""
        self.message_handlers[message_type] = handler
        self.logger.info(f"Registered handler for message type: {message_type}")
    
    def start_receive_loop(self):
        """Start the message receive loop in a separate thread"""
        if self.running:
            self.logger.warning("Receive loop already running")
            return
        
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        self.logger.info("UDP receive loop started")
    
    def stop_receive_loop(self):
        """Stop the message receive loop"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        self.logger.info("UDP receive loop stopped")
    
    def _receive_loop(self):
        """Main receive loop for UDP messages"""
        while self.running:
            try:
                # Check local UDP
                if self.local_connected and self.local_socket:
                    try:
                        data, addr = self.local_socket.recvfrom(1024)
                        self._process_message(data, addr, "local")
                    except socket.timeout:
                        pass
                
                # Check Mission Planner UDP
                if self.mission_planner_connected and self.mission_planner_socket:
                    try:
                        data, addr = self.mission_planner_socket.recvfrom(1024)
                        self._process_message(data, addr, "mission_planner")
                    except socket.timeout:
                        pass
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                self.logger.error(f"Error in receive loop: {e}")
                time.sleep(0.1)
    
    def _process_message(self, data: bytes, addr: tuple, source: str):
        """Process received UDP message"""
        try:
            # Decode message (placeholder for actual deserialization)
            message_str = data.decode('utf-8')
            self.logger.debug(f"Received {source} message from {addr}: {message_str}")
            
            # Parse message and call appropriate handler
            # This is a placeholder - actual message parsing will be implemented
            # when MAVLink integration is added
            
        except Exception as e:
            self.logger.error(f"Failed to process message: {e}")
    
    def cleanup(self):
        """Cleanup UDP resources"""
        try:
            self.stop_receive_loop()
            
            if self.local_socket:
                self.local_socket.close()
                self.local_connected = False
            
            if self.mission_planner_socket:
                self.mission_planner_socket.close()
                self.mission_planner_connected = False
            
            self.logger.info("UDP handler cleaned up")
            
        except Exception as e:
            self.logger.error(f"UDP cleanup failed: {e}")

# Global UDP handler instance
udp_handler = UDPHandler()

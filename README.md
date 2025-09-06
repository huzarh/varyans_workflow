# Raspberry Pi 5 Autonomous Flight System

A professional, modular Python project for autonomous flight control using Raspberry Pi 5, Pixhawk, and Pi Camera v3.

## Phase 1: Servo Initialization and Test Infrastructure

This phase focuses on establishing the project foundation with servo control and state management.

## System Architecture

### Hardware Configuration
- **Raspberry Pi 5**: Companion computer running the autonomous flight system
- **Pixhawk**: Flight controller connected via USB (`/dev/ttyACM0`)
- **Mission Planner**: PC-based ground station connected via UDP (`10.7.225.60:14550`)
- **Pi Camera v3**: Target detection and vision processing
- **Servos**: PWM-controlled servos for payload manipulation

### MAVLink Router Configuration

The system uses MAVLink Router for communication between components:

```
[General]
TcpServerPort=5760
ReportStats=false  # Reduces CPU load on Pi 5

[UartEndpoint pixhawk]
Device=/dev/ttyACM0
Baud=57600
# Primary MAVLink channel for flight control

[UdpEndpoint missionplanner]
Mode=Normal
Address=10.7.225.60
Port=14550
# Mission Planner communication (monitoring only)

[UdpEndpoint localudp]
Mode=Normal
Address=127.0.0.1
Port=14540
# Inter-script communication on Pi
```

### Project Structure

```
pip_varyans/
├── main.py                 # Main entry point
├── requirements.txt        # Python dependencies
├── README.md              # This file
├── mavlink_router.conf    # Generated MAVLink router config
├── config/                # Configuration management
│   ├── __init__.py
│   └── settings.py        # Central configuration
├── core/                  # Core system modules
│   ├── __init__.py
│   ├── state_manager.py   # Multiprocessing state management
│   ├── servo_controller.py # Servo control using pigpio
│   └── logger.py          # Logging configuration
├── flight/                # Flight control modules
│   ├── __init__.py
│   └── mavlink_comm.py    # MAVLink communication
├── vision/                # Vision processing modules
│   ├── __init__.py
│   └── camera_controller.py # Pi Camera v3 control
├── comms/                 # Communication modules
│   ├── __init__.py
│   └── udp_handler.py     # UDP communication
└── data/                  # Data storage
    ├── logs/              # System logs
    └── cache/             # Temporary data
```

## State Management

The system uses `multiprocessing.Manager.dict()` for thread-safe inter-process communication:

```python
from core.state_manager import state_manager

# Read servo state
servo_state = state_manager.get_servo_state(5)
print(f"Servo 5 PWM: {servo_state['pwm']}")

# Update servo state
state_manager.update_servo_state(5, pwm=1800, status="moving")

# Read flight state
flight_state = state_manager.get_flight_state()
print(f"Armed: {flight_state['armed']}")
```

## Servo Control

Servo control is handled by the `ServoController` class using the `pigpio` library:

```python
from core.servo_controller import servo_controller

# Initialize servos
servo_controller.initialize()

# Set servo PWM (1000-2000 microseconds)
servo_controller.set_servo_pwm(5, 1800)

# Set servo angle (-90 to +90 degrees)
servo_controller.set_servo_angle(5, 45.0)

# Test servo movement
servo_controller.test_servo_movement(5, duration=5.0)
```

## Installation

### Prerequisites

1. **Raspberry Pi OS** (latest version)
2. **Python 3.9+**
3. **pigpio daemon** running

### Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd pip_varyans
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Start pigpio daemon**:
   ```bash
   sudo systemctl start pigpiod
   sudo systemctl enable pigpiod
   ```

4. **Configure MAVLink Router** (optional):
   ```bash
   python -c "from config.settings import config; config.save_mavlink_config()"
   ```

## Usage

### Phase 1: Servo Testing

Run the main system to test servo initialization:

```bash
python main.py
```

This will:
1. Initialize servo controllers (servos 5 and 7)
2. Run servo movement tests
3. Monitor system state
4. Display real-time status

### Servo Test Example

```python
from core.servo_controller import servo_controller

# Initialize and test servos
servo_controller.initialize()
servo_controller.test_servo_movement(5, duration=5.0)
servo_controller.test_servo_movement(7, duration=5.0)
```

## Configuration

All system settings are centralized in `config/settings.py`:

- **MAVLink settings**: Ports, IPs, baud rates
- **Servo settings**: PWM values, GPIO pins
- **Camera settings**: Resolution, framerate
- **System settings**: Logging, performance optimization

## Logging

The system provides comprehensive logging:

- **Console output**: Real-time status updates
- **File logging**: Detailed logs in `logs/flight_system.log`
- **Error logging**: Errors in `logs/errors.log`
- **Structured logging**: Servo tests, flight events, vision events

## Safety Features

- **Emergency stop**: All servos return to neutral position
- **Bounds checking**: PWM values are validated
- **Graceful shutdown**: Clean resource cleanup
- **State monitoring**: Real-time system status

## Future Phases

### Phase 2: Vision Processing
- Target detection using Pi Camera v3
- Computer vision algorithms
- Real-time image processing

### Phase 3: Flight Control
- MAVLink integration with Pixhawk
- Autonomous mission execution
- Payload delivery coordination

### Phase 4: Mission Planning
- Waypoint management
- Mission Planner integration
- Flight path optimization

## Development Guidelines

- **Modular design**: Each component is independently testable
- **Thread safety**: All shared state uses proper synchronization
- **Error handling**: Comprehensive exception handling
- **Documentation**: Inline comments and docstrings
- **Logging**: Detailed logging for debugging

## Troubleshooting

### Common Issues

1. **pigpio daemon not running**:
   ```bash
   sudo systemctl start pigpiod
   ```

2. **Permission denied for GPIO**:
   ```bash
   sudo usermod -a -G gpio $USER
   # Log out and back in
   ```

3. **Servo not responding**:
   - Check GPIO connections
   - Verify PWM values (1000-2000)
   - Check power supply

### Debug Mode

Enable debug logging by modifying `config/settings.py`:

```python
log_level: str = "DEBUG"
```

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]

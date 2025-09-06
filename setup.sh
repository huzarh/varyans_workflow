#!/bin/bash
# Setup script for Raspberry Pi 5 Autonomous Flight System

echo "Setting up Raspberry Pi 5 Autonomous Flight System..."

# Update system packages
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install required system packages
echo "Installing system packages..."
sudo apt install -y python3-pip python3-venv pigpio python3-pigpio

# Start and enable pigpio daemon
echo "Starting pigpio daemon..."
sudo systemctl start pigpiod
sudo systemctl enable pigpiod

# Add user to gpio group
echo "Adding user to gpio group..."
sudo usermod -a -G gpio $USER

# Create virtual environment
echo "Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
echo "Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# Create necessary directories
echo "Creating data directories..."
mkdir -p logs cache data

# Set permissions
echo "Setting permissions..."
chmod +x setup.sh
chmod +x test_servo.py

# Generate MAVLink router configuration
echo "Generating MAVLink router configuration..."
python3 -c "from config.settings import config; config.save_mavlink_config()"

echo "Setup completed successfully!"
echo ""
echo "Next steps:"
echo "1. Log out and back in to apply group changes"
echo "2. Activate virtual environment: source venv/bin/activate"
echo "3. Test servo initialization: python test_servo.py"
echo "4. Run main system: python main.py"
echo ""
echo "Note: Make sure servos are connected to GPIO pins 18 and 19"

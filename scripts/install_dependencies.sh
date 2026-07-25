#!/bin/bash
set -e

echo "=================================================="
echo "WRO Future Engineers 2026 - Dependency Installer"
echo "=================================================="

# 1. Update system packages
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# 2. Install required system packages
echo "Installing system dependencies..."
sudo apt install -y python3-pip python3-dev python3-venv \
    i2c-tools libi2c-dev python3-smbus \
    git

# 3. Enable I2C and UART interfaces
echo "Enabling I2C and UART interfaces..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 2   # Disable console on UART
sudo raspi-config nonint do_serial_hw 0   # Enable UART hardware

# 4. Add user to required groups
echo "Adding user to i2c and dialout groups..."
sudo usermod -a -G i2c,dialout $USER

# 5. Navigate to project directory
cd /home/pi/wro_future_engineers_2026

# 6. Create Python virtual environment
echo "Creating Python virtual environment..."
python3 -m venv venv

# 7. Activate and install Python packages
echo "Installing Python packages..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# 8. Install the systemd service
echo "Installing systemd service..."
sudo cp scripts/wro_robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wro_robot.service

echo "=================================================="
echo "Installation complete!"
echo "To start the robot now, run: sudo systemctl start wro_robot.service"
echo "To check status: sudo systemctl status wro_robot.service"
echo "To view logs: sudo journalctl -u wro_robot.service -f"
echo "=================================================="
echo "Reboot recommended: sudo reboot"

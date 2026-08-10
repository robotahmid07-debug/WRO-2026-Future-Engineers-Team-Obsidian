#!/bin/bash
set -e  # Exit on any error

echo "=================================================="
echo "WRO Future Engineers 2026 – Full Auto Installer"
echo "=================================================="

# ------------------------------------------------------------
# 1. System Update & Dependencies
# ------------------------------------------------------------
echo "📦 Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo "📦 Installing system dependencies..."
sudo apt install -y python3-pip python3-dev python3-venv \
    i2c-tools libi2c-dev python3-smbus \
    git curl wget \
    raspi-config

# ------------------------------------------------------------
# 2. Enable I2C and UART interfaces
# ------------------------------------------------------------
echo "🔧 Enabling I2C and UART interfaces..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 2      # Disable console on UART
sudo raspi-config nonint do_serial_hw 0   # Enable UART hardware

# ------------------------------------------------------------
# 3. Add user to required groups
# ------------------------------------------------------------
echo "👤 Adding user to i2c and dialout groups..."
sudo usermod -a -G i2c,dialout $USER

# ------------------------------------------------------------
# 4. Detect RPLIDAR A1 and create udev rule
# ------------------------------------------------------------
echo "🔍 Detecting RPLIDAR A1 and setting up udev rule..."

# Check if LIDAR is connected
if lsusb | grep -q "10c4:ea60"; then
    # Silicon Labs CP210x (common for RPLIDAR A1)
    VID="10c4"
    PID="ea60"
    echo "RPLIDAR A1 detected (Silicon Labs CP210x)."
elif lsusb | grep -q "1a86:7523"; then
    # Alternative USB-to-serial (CH340)
    VID="1a86"
    PID="7523"
    echo "RPLIDAR A1 detected (CH340)."
else
    echo "⚠️ RPLIDAR A1 not detected via USB. Skipping udev rule."
    VID=""
    PID=""
fi

# Create udev rule if we have VID/PID
if [[ -n "$VID" && -n "$PID" ]]; then
    UDEV_RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", SYMLINK+=\"rplidar\", MODE=\"0666\""
    UDEV_FILE="/etc/udev/rules.d/99-rplidar.rules"
    echo "$UDEV_RULE" | sudo tee "$UDEV_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "✅ udev rule created: $UDEV_FILE"
    echo "   LIDAR will appear as /dev/rplidar"
else
    echo "ℹ️ No LIDAR detected. If you connect later, manually create a udev rule."
fi

# ------------------------------------------------------------
# 5. Project directory (assumed to be /home/pi/wro_future_engineers_2026)
# ------------------------------------------------------------
PROJECT_DIR="/home/pi/wro_future_engineers_2026"
if [[ ! -d "$PROJECT_DIR" ]]; then
    echo "❌ Project directory not found: $PROJECT_DIR"
    echo "   Please clone your repository first:"
    echo "   git clone <your-repo-url> $PROJECT_DIR"
    exit 1
fi
cd "$PROJECT_DIR"

# ------------------------------------------------------------
# 6. Python virtual environment
# ------------------------------------------------------------
echo "🐍 Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip

# ------------------------------------------------------------
# 7. Install Python dependencies
# ------------------------------------------------------------
echo "📦 Installing Python packages from requirements.txt..."
if [[ -f "requirements.txt" ]]; then
    pip install -r requirements.txt
else
    echo "⚠️ requirements.txt not found. Installing default packages..."
    pip install pyyaml pyserial rplidar smbus2 RPi.GPIO numpy pyhuskylens pytest flake8
fi

# ------------------------------------------------------------
# 8. Systemd service
# ------------------------------------------------------------
echo "🛠️ Setting up systemd service..."
SERVICE_FILE="/etc/systemd/system/wro_robot.service"
sudo cp scripts/wro_robot.service "$SERVICE_FILE" 2>/dev/null || {
    # If service file doesn't exist in scripts/, create it from template
    cat << EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=WRO Future Engineers 2026 Robot Control
After=network.target local-fs.target multi-user.target
Wants=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$PROJECT_DIR
ExecStart=$PROJECT_DIR/venv/bin/python3 $PROJECT_DIR/main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
Environment="PYTHONUNBUFFERED=1"

[Install]
WantedBy=multi-user.target
EOF
}
sudo systemctl daemon-reload
sudo systemctl enable wro_robot.service

# ------------------------------------------------------------
# 9. Final instructions
# ------------------------------------------------------------
echo "=================================================="
echo "✅ Installation complete!"
echo ""
echo "To start the robot now:"
echo "  sudo systemctl start wro_robot.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status wro_robot.service"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u wro_robot.service -f"
echo ""
echo "If LIDAR udev rule was created, reboot is recommended:"
echo "  sudo reboot"
echo ""
echo "If you changed any settings, reboot to apply them."
echo "=================================================="

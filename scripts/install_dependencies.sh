#!/bin/bash
set -e  # Exit on any error

# ================================================================
# WRO Future Engineers 2026 – Fully Automated Installer
# ================================================================
# This script installs everything needed to run the robot.
# It auto‑detects the project path, RPLIDAR, and sets up the
# systemd service for auto‑start on boot.
# ================================================================

# ---- Colours for pretty output ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Colour

# ---- Helper functions ----
print_step() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}▶ $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_ok() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warn() {
    echo -e "${YELLOW}⚠️ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "ℹ️ $1"
}

# ------------------------------------------------------------
# 0. Detect project directory (where this script is located)
# ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"
print_ok "Project directory: $PROJECT_DIR"
print_info "Script directory: $SCRIPT_DIR"

# ------------------------------------------------------------
# 1. System Update & Dependencies
# ------------------------------------------------------------
print_step "1. Updating system packages"
sudo apt update
sudo apt upgrade -y

print_step "2. Installing system dependencies"
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    i2c-tools \
    libi2c-dev \
    python3-smbus \
    git \
    curl \
    wget \
    raspi-config \
    usbutils \
    pkg-config

# ------------------------------------------------------------
# 2. Enable I2C and UART interfaces
# ------------------------------------------------------------
print_step "3. Enabling I2C and UART interfaces"
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 2      # Disable console on UART
sudo raspi-config nonint do_serial_hw 0   # Enable UART hardware

# ------------------------------------------------------------
# 3. Add user to required groups
# ------------------------------------------------------------
print_step "4. Adding user to i2c and dialout groups"
sudo usermod -a -G i2c,dialout "$USER"
print_ok "User added to groups (requires logout/login to take effect)"

# ------------------------------------------------------------
# 4. Detect RPLIDAR A1 and create udev rule
# ------------------------------------------------------------
print_step "5. Detecting RPLIDAR A1 and setting up udev rule"

# Find LIDAR USB vendor/product ID
VID=""
PID=""

# Check common RPLIDAR USB IDs
if lsusb | grep -qi "10c4:ea60"; then
    VID="10c4"
    PID="ea60"
    print_ok "RPLIDAR A1 detected (Silicon Labs CP210x)"
elif lsusb | grep -qi "1a86:7523"; then
    VID="1a86"
    PID="7523"
    print_ok "RPLIDAR A1 detected (CH340)"
elif lsusb | grep -qi "10c4:ea80"; then
    VID="10c4"
    PID="ea80"
    print_ok "RPLIDAR detected (Silicon Labs CP210x variant)"
else
    print_warn "RPLIDAR A1 not detected via USB. Skipping udev rule creation."
    print_info "If you connect it later, run: lsusb to find the ID, then manually create the udev rule."
fi

# Create udev rule if we have VID/PID
if [[ -n "$VID" && -n "$PID" ]]; then
    UDEV_RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", SYMLINK+=\"rplidar\", MODE=\"0666\""
    UDEV_FILE="/etc/udev/rules.d/99-rplidar.rules"
    echo "$UDEV_RULE" | sudo tee "$UDEV_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    print_ok "udev rule created: $UDEV_FILE"
    print_info "LIDAR will appear as /dev/rplidar"
else
    print_warn "No LIDAR detected. If you connect later, manually create a udev rule."
fi

# ------------------------------------------------------------
# 5. Python virtual environment
# ------------------------------------------------------------
print_step "6. Creating Python virtual environment"
if [[ -d "venv" ]]; then
    print_warn "Virtual environment already exists. Recreating..."
    rm -rf venv
fi
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
print_ok "Virtual environment created at $PROJECT_DIR/venv"

# ------------------------------------------------------------
# 6. Install Python dependencies
# ------------------------------------------------------------
print_step "7. Installing Python packages"
if [[ -f "requirements.txt" ]]; then
    print_info "Installing from requirements.txt"
    pip install -r requirements.txt
else
    print_warn "requirements.txt not found. Installing default packages..."
    pip install \
        pyyaml \
        pyserial \
        rplidar \
        smbus2 \
        RPi.GPIO \
        numpy \
        pyhuskylens \
        pytest \
        flake8 \
        opencv-python \
        matplotlib
fi
print_ok "Python packages installed"

# ------------------------------------------------------------
# 7. Systemd service
# ------------------------------------------------------------
print_step "8. Setting up systemd service"

# Create the service file from template
SERVICE_FILE="/etc/systemd/system/wro_robot.service"
cat << EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=WRO Future Engineers 2026 Robot Control
After=network.target local-fs.target multi-user.target
Wants=network.target

[Service]
Type=simple
User=$USER
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

sudo systemctl daemon-reload
sudo systemctl enable wro_robot.service
print_ok "Systemd service installed and enabled"

# ------------------------------------------------------------
# 8. Check if the service file exists in scripts/ and copy it
# ------------------------------------------------------------
if [[ -f "$SCRIPT_DIR/wro_robot.service" ]]; then
    print_info "Service file found in $SCRIPT_DIR – copying to /etc/systemd/system/"
    sudo cp "$SCRIPT_DIR/wro_robot.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable wro_robot.service
    print_ok "Updated service from scripts/ directory"
fi

# ------------------------------------------------------------
# 9. Fix LIDAR port in main.py (optional)
# ------------------------------------------------------------
print_step "9. Checking LIDAR port configuration"
if [[ -f "main.py" ]]; then
    if grep -q 'port=.*/dev/ttyUSB' main.py; then
        print_warn "main.py uses /dev/ttyUSB* – consider changing to /dev/rplidar for consistency with udev rule."
        print_info "You can edit main.py and change LIDAR port to: port='/dev/rplidar'"
    fi
fi

# ------------------------------------------------------------
# 10. Final instructions
# ------------------------------------------------------------
echo ""
echo "================================================================"
echo -e "${GREEN}✅ Installation complete!${NC}"
echo "================================================================"
echo ""
echo -e "${BLUE}To start the robot now:${NC}"
echo "  sudo systemctl start wro_robot.service"
echo ""
echo -e "${BLUE}To check status:${NC}"
echo "  sudo systemctl status wro_robot.service"
echo ""
echo -e "${BLUE}To view logs:${NC}"
echo "  sudo journalctl -u wro_robot.service -f"
echo ""
echo -e "${BLUE}To stop the robot:${NC}"
echo "  sudo systemctl stop wro_robot.service"
echo ""
echo -e "${YELLOW}If the LIDAR udev rule was created, reboot is recommended:${NC}"
echo "  sudo reboot"
echo ""
echo -e "${YELLOW}If you want to run the robot manually (without service):${NC}"
echo "  source venv/bin/activate"
echo "  python main.py"
echo ""
echo "================================================================"

"""
Serial communication bridge between Raspberry Pi and ESP32‑S3.
Handles UART with JSON framing and XOR checksum.

Features:
  - Send commands (motor, steer) to ESP32.
  - Receive sensor data (ultrasonic, IMU) from ESP32.
  - Verify XOR checksum using raw payload substring (avoids formatting issues).
  - Maintains a shared `latest_sensor_data` dictionary for multiple consumers.
"""

import json
import time
import logging
import serial
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class SerialBridge:
    def __init__(self, port: str = "/dev/ttyS0", baudrate: int = 115200,
                 timeout: float = 0.1):
        """
        Initialize UART connection.

        Args:
            port: Serial device (e.g., '/dev/ttyS0' or '/dev/ttyUSB0').
            baudrate: Communication speed (must match ESP32).
            timeout: Read timeout in seconds.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        # Open the port immediately
        self._open_port()

        # Shared sensor state (latest data from ESP32)
        self.latest_sensor_data: Optional[Dict[str, Any]] = None
        self.latest_timestamp: float = 0.0

        # Internal buffer for incomplete lines
        self._buffer = ""

    def _open_port(self):
        """Internal method to open serial port."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info(f"SerialBridge opened on {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            self.ser = None

    def open(self):
        """
        Public open method (for compatibility with main.py).
        If port is closed, re‑open; otherwise do nothing.
        """
        if self.ser is None or not self.ser.is_open:
            self._open_port()
        else:
            logger.debug("Serial port already open")

    def _verify_checksum(self, raw_bytes: bytes) -> bool:
        """
        Verify XOR checksum without reconstructing JSON.

        Extracts the payload before ',"checksum":' and compares its XOR
        with the sent checksum. This avoids float formatting differences
        between Python and Arduino (e.g., '129.00' vs '129.0').

        Returns:
            True if checksum matches, False otherwise.
        """
        try:
            # Locate the start of the checksum field
            idx = raw_bytes.find(b',"checksum":')
            if idx == -1:
                logger.debug("Checksum field not found")
                return False

            # Payload is everything before that marker
            payload = raw_bytes[:idx]

            # Compute XOR of payload
            computed = 0
            for b in payload:
                computed ^= b

            # Extract the sent checksum (digits after the colon)
            start = idx + len(b',"checksum":')
            end = start
            while end < len(raw_bytes) and raw_bytes[end] in b'0123456789':
                end += 1

            if start == end:
                logger.debug("No checksum digits found")
                return False

            sent_checksum = int(raw_bytes[start:end])
            return computed == sent_checksum

        except Exception as e:
            logger.debug(f"Checksum verification failed: {e}")
            return False

    def _read_line(self) -> Optional[bytes]:
        """Read one complete line (terminated with '\\n') from serial."""
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            # Read until newline or timeout
            line = self.ser.readline()
            if line and line.endswith(b'\n'):
                return line.strip()
            return None
        except Exception as e:
            logger.error(f"Serial read error: {e}")
            return None

    def receive(self, block: bool = False) -> Optional[Dict[str, Any]]:
        """
        Read and parse one message from ESP32.

        Args:
            block: If True, wait indefinitely for a message (not recommended).

        Returns:
            Parsed JSON dictionary if valid, or None if no message/error.
        """
        raw = self._read_line()
        if raw is None:
            return None

        # Verify checksum before parsing
        if not self._verify_checksum(raw):
            logger.warning("Checksum mismatch, discarding message")
            return None

        # Parse JSON
        try:
            # Convert bytes to string, remove checksum field to avoid parsing conflicts
            # We'll find the actual JSON object by stripping the checksum part
            msg_str = raw.decode('utf-8', errors='ignore')
            # Remove the checksum field to parse cleanly
            # Find the start of checksum field and cut it
            idx = msg_str.find(',"checksum":')
            if idx != -1:
                # Find the closing brace or newline
                end = msg_str.find('}', idx)
                if end != -1:
                    # Remove everything from ', "checksum":...' to the end
                    clean = msg_str[:idx] + msg_str[end:]
                else:
                    clean = msg_str[:idx]
            else:
                clean = msg_str

            # Now parse JSON
            data = json.loads(clean)
            logger.debug(f"Received: {data}")

            # Update shared sensor state if it's sensor data
            if data.get('type') == 'sensor_data' and 'data' in data:
                self.latest_sensor_data = data['data']
                self.latest_timestamp = time.time()

            return data

        except json.JSONDecodeError as e:
            logger.warning(f"JSON decode error: {e} | Raw: {raw[:100]}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error in receive: {e}")
            return None

    def send(self, command: Dict[str, Any]) -> bool:
        """
        Send a JSON command to ESP32 (e.g., {"motor": 150, "steer": 45}).

        Returns:
            True if sent successfully, False otherwise.
        """
        if self.ser is None or not self.ser.is_open:
            logger.error("Serial port not open")
            return False

        try:
            json_str = json.dumps(command) + "\n"
            written = self.ser.write(json_str.encode())
            if written == len(json_str):
                logger.debug(f"Sent: {json_str.strip()}")
                return True
            else:
                logger.error(f"Write incomplete: {written}/{len(json_str)}")
                return False
        except Exception as e:
            logger.error(f"Send error: {e}")
            return False

    def get_latest_sensor_data(self) -> Optional[Dict[str, Any]]:
        """
        Return the most recent sensor data without reading from serial.

        This allows multiple consumers (e.g., state_machine, emergency_shield)
        to access the same data without interfering with each other.
        """
        return self.latest_sensor_data

    def close(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("SerialBridge closed")

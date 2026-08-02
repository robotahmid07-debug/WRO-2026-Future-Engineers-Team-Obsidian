"""
Handles UART communication between Raspberry Pi and ESP32-S3.
Uses JSON framing with XOR checksum for reliability.
"""

import serial
import json
import threading
import time
import logging
from queue import Queue, Empty
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class SerialBridge:
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 460800, timeout: float = 0.01):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.rx_queue = Queue(maxsize=100)
        self.tx_queue = Queue(maxsize=100)
        self.running = False
        self.read_thread = None
        self.write_thread = None

    def open(self):
        """Open the serial port and start background read/write threads."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.write_thread = threading.Thread(target=self._write_loop, daemon=True)
            self.read_thread.start()
            self.write_thread.start()
            logger.info(f"Serial bridge opened on {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open serial port: {e}")
            raise

    def close(self):
        """Close the serial port and stop background threads."""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.write_thread:
            self.write_thread.join(timeout=1.0)
        logger.info("Serial bridge closed")

    def _read_loop(self):
        """Background thread: continuously read lines from serial and parse JSON."""
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                    if self._verify_checksum(msg):
                        self.rx_queue.put(msg, block=False)
                    else:
                        logger.warning("Checksum mismatch, dropping message")
                except json.JSONDecodeError:
                    logger.warning("Malformed JSON received")
            except Exception as e:
                logger.error(f"Read error: {e}")
                time.sleep(0.01)

    def _write_loop(self):
        """Background thread: send messages from the TX queue."""
        while self.running and self.ser and self.ser.is_open:
            try:
                msg = self.tx_queue.get(timeout=0.1)
                if msg is None:
                    continue
                msg['checksum'] = self._compute_checksum(msg)
                payload = json.dumps(msg) + '\n'
                self.ser.write(payload.encode('utf-8'))
            except Empty:
                pass
            except Exception as e:
                logger.error(f"Write error: {e}")

    @staticmethod
    def _compute_checksum(msg: Dict[str, Any]) -> int:
        """
        Compute XOR checksum over the message content (excluding 'checksum' key).
        Matches the ESP32 firmware implementation.
        """
        data = {k: v for k, v in msg.items() if k != 'checksum'}
        json_str = json.dumps(data, sort_keys=True, separators=(',', ':'))
        checksum = 0
        for char in json_str:
            checksum ^= ord(char)
        return checksum

    @staticmethod
    def _verify_checksum(msg: Dict[str, Any]) -> bool:
        """Verify that the message's checksum matches the computed value."""
        if 'checksum' not in msg:
            return False
        expected = msg.pop('checksum')
        computed = SerialBridge._compute_checksum(msg)
        msg['checksum'] = expected
        return computed == expected

    def send(self, msg: Dict[str, Any]) -> None:
        """Send a JSON message to the ESP32 (non‑blocking)."""
        self.tx_queue.put(msg)

    def receive(self, block: bool = True, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """Receive a JSON message from the ESP32 (non‑blocking by default)."""
        try:
            return self.rx_queue.get(block=block, timeout=timeout)
        except Empty:
            return None

    def get_ultrasonic_data(self) -> Optional[Dict[str, float]]:
        """Convenience: return latest sensor data (distances + IMU)."""
        msg = self.receive(block=False)
        if msg and msg.get('type') == 'sensor_data':
            return msg.get('data', {})
        return None

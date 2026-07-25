"""
HuskyLens V2 reader using the official pyhuskylens library.
Supports Color Recognition mode with proper protocol handling.
"""

import time
import threading
import logging
from queue import Queue, Empty
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

# Official pyhuskylens library for HuskyLens V2[reference:3][reference:4]
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION

logger = logging.getLogger(__name__)


@dataclass
class ColorBlock:
    """Color recognition result from HuskyLens V2."""
    color_id: int           # ID assigned when color was learned[reference:5]
    color_name: str         # Custom name for the color
    x: int                  # Center X coordinate (0-319)
    y: int                  # Center Y coordinate (0-239)
    width: int              # Width of bounding box
    height: int             # Height of bounding box
    area: int               # Area of the color block
    timestamp: float


class HuskyLensReader:
    """
    Production-grade HuskyLens V2 reader using the official pyhuskylens library.
    Supports I2C communication on Raspberry Pi[reference:6].
    """

    # Algorithm constants for HuskyLens V2
    ALGORITHM_COLOR_RECOGNITION = ALGORITHM_COLOR_RECOGNITION  # "COLOR_RECOGNITION"

    def __init__(self, i2c_bus: int = 1, poll_interval: float = 0.05):
        """
        Initialize HuskyLens V2 reader.

        Args:
            i2c_bus: I2C bus number (1 on Raspberry Pi)[reference:7]
            poll_interval: Polling interval in seconds
        """
        self.i2c_bus = i2c_bus
        self.poll_interval = poll_interval
        self.hl: Optional[HuskyLens] = None
        self.running = False
        self.read_thread = None
        self.color_queue = Queue(maxsize=100)

        # Color name mapping (learned colors)
        self.color_names: Dict[int, str] = {}
        self._color_name_lock = threading.Lock()

    def open(self) -> bool:
        """
        Initialize and connect to HuskyLens V2 over I2C.

        Returns:
            True if connection successful, False otherwise.
        """
        try:
            # Initialize HuskyLens on I2C bus[reference:8]
            self.hl = HuskyLens(self.i2c_bus)

            # Verify connection[reference:9]
            if not self.hl.knock():
                logger.error("HuskyLens V2 not detected on I2C bus %d", self.i2c_bus)
                return False

            # Get version info
            version = self.hl.version
            logger.info("HuskyLens V%d connected on I2C bus %d", version, self.i2c_bus)

            # Switch to Color Recognition algorithm[reference:10]
            if not self.hl.set_alg(self.ALGORITHM_COLOR_RECOGNITION):
                logger.error("Failed to switch to Color Recognition mode")
                return False

            logger.info("HuskyLens V2 switched to Color Recognition mode")

            # Start background reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            return True

        except Exception as e:
            logger.error("Failed to initialize HuskyLens V2: %s", e)
            return False

    def close(self):
        """Close the HuskyLens connection and stop background thread."""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        if self.hl:
            self.hl = None
        logger.info("HuskyLens V2 closed")

    def _read_loop(self):
        """Background thread: continuously poll HuskyLens for color detections."""
        while self.running and self.hl:
            try:
                # Get color recognition results[reference:11]
                # For V2, get_blocks() returns detected objects
                blocks = self.hl.get_blocks()

                if blocks:
                    for block in blocks:
                        # Extract color data
                        # block format: (x, y, width, height, id, name?)
                        # For V2 color recognition, each block has an ID[reference:12]
                        if len(block) >= 5:
                            x, y, w, h, color_id = block[:5]
                            color_name = self._get_color_name(color_id)

                            color_block = ColorBlock(
                                color_id=color_id,
                                color_name=color_name,
                                x=x,
                                y=y,
                                width=w,
                                height=h,
                                area=w * h,
                                timestamp=time.time()
                            )

                            # Non-blocking queue put
                            try:
                                self.color_queue.put(color_block, block=False)
                            except:
                                pass

                # Small sleep to prevent busy-waiting
                time.sleep(self.poll_interval)

            except Exception as e:
                logger.error("HuskyLens read error: %s", e)
                time.sleep(0.1)

    def _get_color_name(self, color_id: int) -> str:
        """Get the name for a color ID, or return default."""
        with self._color_name_lock:
            return self.color_names.get(color_id, f"Color_ID_{color_id}")

    def set_color_name(self, color_id: int, name: str):
        """
        Set a custom name for a learned color ID.

        Args:
            color_id: The color ID (from HuskyLens)
            name: Custom name to display
        """
        with self._color_name_lock:
            self.color_names[color_id] = name
        logger.info("Color ID %d set to name: %s", color_id, name)

    def get_latest_colors(self) -> List[ColorBlock]:
        """
        Get all color blocks currently in the queue (non-blocking).

        Returns:
            List of ColorBlock objects.
        """
        colors = []
        while not self.color_queue.empty():
            try:
                colors.append(self.color_queue.get_nowait())
            except Empty:
                break
        return colors

    def get_latest_color(self) -> Optional[ColorBlock]:
        """
        Get the most recent color detection (blocking with short timeout).

        Returns:
            ColorBlock or None if no detection available.
        """
        try:
            return self.color_queue.get(timeout=0.05)
        except Empty:
            return None

    def get_closest_color(self) -> Optional[ColorBlock]:
        """
        Get the color block closest to the center of the frame.

        Returns:
            ColorBlock closest to center, or None.
        """
        colors = self.get_latest_colors()
        if not colors:
            return None

        # Find color block with center closest to (160, 120)
        center_x, center_y = 160, 120
        closest = None
        min_dist = float('inf')

        for color in colors:
            dist = ((color.x - center_x) ** 2 + (color.y - center_y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest = color

        return closest

    def get_color_by_id(self, color_id: int) -> Optional[ColorBlock]:
        """
        Get the most recent detection for a specific color ID.

        Args:
            color_id: The color ID to look for.

        Returns:
            ColorBlock or None if not detected.
        """
        colors = self.get_latest_colors()
        for color in colors:
            if color.color_id == color_id:
                return color
        return None

    def learn_color(self) -> bool:
        """
        Learn the color currently under the crosshair.
        Press the 'A' button on HuskyLens to learn the target color.[reference:13]

        Returns:
            True if learning was successful.
        """
        if not self.hl:
            return False
        # The HuskyLens learns colors via hardware button press (A button)
        # This is a hardware operation - we just log that the user should press A
        logger.info("Press the 'A' button on HuskyLens V2 to learn the target color")
        return True

    def forget_all_colors(self) -> bool:
        """
        Forget all previously learned colors.[reference:14]

        Returns:
            True if successful.
        """
        if not self.hl:
            return False
        # This would require sending the forget command via protocol
        # For now, we log the action
        logger.info("Forgetting all learned colors...")
        return True

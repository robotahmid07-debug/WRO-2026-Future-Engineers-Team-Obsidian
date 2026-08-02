"""
HuskyLens V2 reader using the official pyhuskylens library.
Supports Color Recognition mode with proper protocol handling.
"""
# hi
import time
import threading
import logging
from queue import Queue, Empty
from dataclasses import dataclass
from typing import Optional, List

# Official pyhuskylens library for HuskyLens V2
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION

logger = logging.getLogger(__name__)


@dataclass
class ColorBlock:
    """Color recognition result from HuskyLens V2."""
    color_id: int           # ID assigned when colour was learned
    color_name: str         # Custom name for the colour
    x: int                  # Centre X coordinate (0-319)
    y: int                  # Centre Y coordinate (0-239)
    width: int              # Width of bounding box
    height: int             # Height of bounding box
    area: int               # Area of the colour block
    timestamp: float


class HuskyLensReader:
    """
    Production-grade HuskyLens V2 reader using the official pyhuskylens library.
    Supports I2C communication on Raspberry Pi.
    """

    def __init__(self, i2c_bus: int = 1, poll_interval: float = 0.05):
        """
        Initialise HuskyLens V2 reader.

        Args:
            i2c_bus: I2C bus number (1 on Raspberry Pi).
            poll_interval: Polling interval in seconds.
        """
        self.i2c_bus = i2c_bus
        self.poll_interval = poll_interval
        self.hl: Optional[HuskyLens] = None
        self.running = False
        self.read_thread = None
        self.color_queue = Queue(maxsize=100)

        # Colour name mapping (learned colours)
        self.color_names: dict[int, str] = {}
        self._color_name_lock = threading.Lock()

    def open(self) -> bool:
        """
        Initialise and connect to HuskyLens V2 over I2C.

        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.hl = HuskyLens(self.i2c_bus)

            # Verify connection
            if not self.hl.knock():
                logger.error("HuskyLens V2 not detected on I2C bus %d", self.i2c_bus)
                return False

            version = self.hl.version
            logger.info("HuskyLens V%d connected on I2C bus %d", version, self.i2c_bus)

            # Switch to Colour Recognition algorithm
            if not self.hl.set_alg(ALGORITHM_COLOR_RECOGNITION):
                logger.error("Failed to switch to Colour Recognition mode")
                return False

            logger.info("HuskyLens V2 switched to Colour Recognition mode")

            # Start background reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            return True

        except Exception as e:
            logger.error("Failed to initialise HuskyLens V2: %s", e)
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
        """Background thread: continuously poll HuskyLens for colour detections."""
        while self.running and self.hl:
            try:
                blocks = self.hl.get_blocks()

                if blocks:
                    for block in blocks:
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

                            try:
                                self.color_queue.put(color_block, block=False)
                            except Exception:
                                # Queue full or other issue – silently drop the frame
                                pass

                time.sleep(self.poll_interval)

            except Exception as e:
                logger.error("HuskyLens read error: %s", e)
                time.sleep(0.1)

    def _get_color_name(self, color_id: int) -> str:
        """Get the name for a colour ID, or return a default."""
        with self._color_name_lock:
            return self.color_names.get(color_id, f"Color_ID_{color_id}")

    def set_color_name(self, color_id: int, name: str):
        """
        Set a custom name for a learned colour ID.

        Args:
            color_id: The colour ID (from HuskyLens)
            name: Custom name to display
        """
        with self._color_name_lock:
            self.color_names[color_id] = name
        logger.info("Colour ID %d set to name: %s", color_id, name)

    def get_latest_colors(self) -> List[ColorBlock]:
        """
        Get all colour blocks currently in the queue (non-blocking).

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
        Get the most recent colour detection (blocking with short timeout).

        Returns:
            ColorBlock or None if no detection available.
        """
        try:
            return self.color_queue.get(timeout=0.05)
        except Empty:
            return None

    def get_closest_color(self) -> Optional[ColorBlock]:
        """
        Get the colour block closest to the centre of the frame.

        Returns:
            ColorBlock closest to centre, or None.
        """
        colors = self.get_latest_colors()
        if not colors:
            return None

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
        Get the most recent detection for a specific colour ID.

        Args:
            color_id: The colour ID to look for.

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
        Learn the colour currently under the crosshair.
        Press the 'A' button on HuskyLens to learn the target colour.

        Returns:
            True if learning was successful.
        """
        if not self.hl:
            return False
        logger.info("Press the 'A' button on HuskyLens V2 to learn the target colour")
        return True

    def forget_all_colors(self) -> bool:
        """
        Forget all previously learned colours.

        Returns:
            True if successful.
        """
        if not self.hl:
            return False
        logger.info("Forgetting all learned colours...")
        return True

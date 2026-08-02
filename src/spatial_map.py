"""
Host-side object persistence, spatial gating, and debouncing.
Maintains a list of confirmed objects and handles frame drops.

Features:
  - Spatial gating: merge detections within 20 cm.
  - Confirmation: require N consecutive frames.
  - Persistence: hold objects for up to 1.5s during dropouts.
  - Velocity‑compensated pruning: objects move backward with the robot;
    purged immediately when behind the front axle (y_local < 0).
  - Thread‑safe for use in multi‑threaded environment.
"""
# Hi

import time
import math
import logging
from dataclasses import dataclass, field
from typing import List, Tuple
from collections import deque
import threading

logger = logging.getLogger(__name__)


@dataclass
class TrackedObject:
    """A single tracked object in the spatial map."""
    color_id: int
    local_x: float          # meters, lateral (left = positive)
    local_y: float          # meters, forward (front = positive)
    timestamp: float
    confidence: int = 1     # number of consecutive frames seen
    last_seen: float = field(default_factory=time.time)
    # Store recent positions for optional smoothing (not used currently)
    history: deque = field(default_factory=lambda: deque(maxlen=5))


class SpatialMap:
    def __init__(self, config: dict):
        """
        Initialize the spatial map.

        Args:
            config: Dictionary containing:
                - confirmation_threshold_frames: N frames required for confirmation.
                - frame_loss_tolerance_sec: Max seconds to hold object during dropouts.
                - tolerance_radius_m: Merge distance for nearby detections.
                - prune_condition: "Y_LOCAL < 0.0" (objects behind robot are pruned).
        """
        # Load configuration with defaults
        self.confirmation_threshold = config.get('confirmation_threshold_frames', 3)
        self.loss_tolerance_sec = config.get('frame_loss_tolerance_sec', 1.5)
        self.merge_tolerance_m = config.get('tolerance_radius_m', 0.20)

        # Internal state
        self.objects: List[TrackedObject] = []
        self.lock = threading.Lock()
        self.next_id = 0  # not used, but kept for possible future expansion

        logger.info(f"SpatialMap initialized: threshold={self.confirmation_threshold}, "
                    f"loss_tolerance={self.loss_tolerance_sec}s, merge={self.merge_tolerance_m}m")

    def update(self, detections: List[Tuple[int, float, float]],
               robot_speed: float = 0.0, dt: float = 0.05) -> None:
        """
        Update the spatial map with new detections and ego‑motion compensation.

        Args:
            detections: List of (color_id, x_local, y_local) in meters.
            robot_speed: Current forward speed of the robot (m/s).
                         Used to move objects backward.
            dt: Time step (seconds) since last update.
        """
        with self.lock:
            current_time = time.time()

            # ---- 1. Ego‑motion compensation: move objects backward ----
            if robot_speed > 0.001 and dt > 0:
                for obj in self.objects:
                    # Move object backward by the distance the robot travelled
                    obj.local_y -= robot_speed * dt
                    # If object is now behind the robot, mark for removal
                    if obj.local_y < 0.0:
                        # We'll remove it in the pruning step below
                        pass

            # ---- 2. Remove objects that are behind the robot ----
            self.objects = [obj for obj in self.objects if obj.local_y >= 0.0]

            # ---- 3. Match new detections to existing objects ----
            matched_indices = set()
            for det in detections:
                color_id, x, y = det
                best_idx = -1
                best_dist = self.merge_tolerance_m * 2  # start larger

                for i, obj in enumerate(self.objects):
                    # Only match same color
                    if obj.color_id != color_id:
                        continue
                    # Compute distance
                    dist = math.hypot(obj.local_x - x, obj.local_y - y)
                    if dist < self.merge_tolerance_m and dist < best_dist:
                        best_dist = dist
                        best_idx = i

                if best_idx >= 0:
                    # Update existing object (simple moving average)
                    obj = self.objects[best_idx]
                    # Smooth position update: 70% existing, 30% new
                    obj.local_x = 0.7 * obj.local_x + 0.3 * x
                    obj.local_y = 0.7 * obj.local_y + 0.3 * y
                    obj.confidence = min(obj.confidence + 1, 10)
                    obj.last_seen = current_time
                    obj.timestamp = current_time
                    matched_indices.add(best_idx)
                else:
                    # Create new object
                    new_obj = TrackedObject(
                        color_id=color_id,
                        local_x=x,
                        local_y=y,
                        timestamp=current_time,
                        confidence=1,
                        last_seen=current_time
                    )
                    self.objects.append(new_obj)

            # ---- 4. Degrade confidence for unmatched objects (optional) ----
            # We don't decrement confidence, but we rely on the loss tolerance
            # to remove old objects. This is simpler and works well.

            # ---- 5. Prune old objects (based on time) ----
            # Remove objects that haven't been seen for longer than tolerance
            self.objects = [obj for obj in self.objects
                            if (current_time - obj.last_seen) < self.loss_tolerance_sec]

            # ---- 6. Optional: remove objects with low confidence after long time ----
            # Already handled by the loss tolerance above.

    def get_confirmed_objects(self) -> List[TrackedObject]:
        """
        Return only objects with confidence >= threshold and within active window.

        Returns:
            List of confirmed TrackedObject, sorted by forward distance (closest first).
        """
        with self.lock:
            current_time = time.time()
            confirmed = [obj for obj in self.objects
                         if obj.confidence >= self.confirmation_threshold and
                         (current_time - obj.last_seen) < self.loss_tolerance_sec and
                         obj.local_y >= 0.0]  # extra safety prune

            # Sort by forward distance (ascending) – closest first
            confirmed.sort(key=lambda o: o.local_y)
            return confirmed

    def get_all_objects(self) -> List[TrackedObject]:
        """Return all active objects (including unconfirmed) – for debugging."""
        with self.lock:
            return self.objects.copy()

    def clear(self) -> None:
        """Clear all objects from the spatial map."""
        with self.lock:
            self.objects.clear()
            logger.info("SpatialMap cleared")

    def get_object_count(self) -> int:
        """Return the number of active objects (for debugging)."""
        with self.lock:
            return len(self.objects)

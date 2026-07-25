"""
Host-side object persistence, spatial gating, and debouncing.
Maintains a list of confirmed objects and handles frame drops.
"""

import time
import math
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
from collections import deque
import logging

logger = logging.getLogger(__name__)


@dataclass
class TrackedObject:
    color_id: int
    local_x: float  # meters, relative to robot (forward = x)
    local_y: float  # meters, lateral (left = y)
    timestamp: float
    confidence: int = 0  # number of consecutive frames seen
    last_seen: float = field(default_factory=time.time)
    # For merging, we also store recent positions for smoothing (optional)
    history: deque = field(default_factory=lambda: deque(maxlen=5))


class SpatialMap:
    def __init__(self, config: dict):
        self.confirmation_threshold = config.get('confirmation_threshold_frames', 3)
        self.loss_tolerance_sec = config.get('frame_loss_tolerance_sec', 1.5)
        self.merge_tolerance_m = config.get('tolerance_radius_m', 0.20)
        self.prune_condition_y_less_than = config.get('prune_condition', 'Y_LOCAL < 0.0')  # we'll parse
        self.objects: List[TrackedObject] = []
        self.lock = threading.Lock()
        self.next_id = 0

    def update(self, detections: List[Tuple[int, float, float]]) -> None:
        """
        detections: list of (color_id, x_local, y_local) in meters.
        """
        with self.lock:
            # 1. Match detections to existing objects
            matched_indices = set()
            for det in detections:
                color_id, x, y = det
                best_idx = -1
                best_dist = self.merge_tolerance_m * 2  # start larger
                for i, obj in enumerate(self.objects):
                    if obj.color_id != color_id:
                        continue
                    dist = math.hypot(obj.local_x - x, obj.local_y - y)
                    if dist < self.merge_tolerance_m and dist < best_dist:
                        best_dist = dist
                        best_idx = i
                if best_idx >= 0:
                    # Update existing object
                    obj = self.objects[best_idx]
                    obj.local_x = (obj.local_x + x) / 2  # simple moving average
                    obj.local_y = (obj.local_y + y) / 2
                    obj.confidence = min(obj.confidence + 1, 10)
                    obj.last_seen = time.time()
                    obj.timestamp = time.time()
                    matched_indices.add(best_idx)
                else:
                    # New object
                    new_obj = TrackedObject(
                        color_id=color_id,
                        local_x=x,
                        local_y=y,
                        timestamp=time.time(),
                        confidence=1,
                        last_seen=time.time()
                    )
                    self.objects.append(new_obj)

            # 2. Increment confidence for matched; for unmatched, confidence stays same but we will handle below

            # 3. Handle objects that were not matched in this update: their confidence decays? 
            # Actually we require N consecutive frames; if not seen, confidence does not decrease, but we track last_seen.
            # We will prune after loss tolerance.

            # 4. Promote objects to confirmed if confidence >= threshold
            # We'll just keep them; later we can filter.

            # 5. Prune objects that are behind the robot (y_local < 0) and not recently seen?
            # According to spec: prune when y_local < 0.0.
            # Also prune if last_seen too old.
            current_time = time.time()
            self.objects = [obj for obj in self.objects
                            if obj.local_y >= 0.0 and
                            (current_time - obj.last_seen) < self.loss_tolerance_sec]

    def get_confirmed_objects(self) -> List[TrackedObject]:
        """Return only objects with confidence >= threshold and within active window."""
        with self.lock:
            current_time = time.time()
            confirmed = [obj for obj in self.objects
                         if obj.confidence >= self.confirmation_threshold and
                         (current_time - obj.last_seen) < self.loss_tolerance_sec]
            # Sort by forward distance (x) ascending (closest first)
            confirmed.sort(key=lambda o: o.local_x)
            return confirmed

"""
Unit tests for spatial map fusion (spatial gating, persistence, pruning).
"""

import pytest
import time
from src.spatial_map import SpatialMap


def test_spatial_gating_merge():
    """Test that detections within tolerance are merged."""
    config = {
        'confirmation_threshold_frames': 2,
        'frame_loss_tolerance_sec': 1.0,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    # Add first detection
    sm.update([(1, 0.5, 0.3)])
    assert len(sm.objects) == 1
    assert sm.objects[0].confidence == 1

    # Add second detection within merge tolerance
    sm.update([(1, 0.55, 0.28)])
    assert len(sm.objects) == 1
    assert sm.objects[0].confidence == 2

    # Should be confirmed (threshold = 2)
    confirmed = sm.get_confirmed_objects()
    assert len(confirmed) == 1


def test_spatial_gating_no_merge():
    """Test that detections outside tolerance create new objects."""
    config = {
        'confirmation_threshold_frames': 2,
        'frame_loss_tolerance_sec': 1.0,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    sm.update([(1, 0.5, 0.3)])
    sm.update([(1, 1.0, 0.8)])  # > 0.20m away
    assert len(sm.objects) == 2


def test_persistence_pruning():
    """Test that objects are pruned after loss tolerance."""
    config = {
        'confirmation_threshold_frames': 2,
        'frame_loss_tolerance_sec': 0.5,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    sm.update([(1, 0.5, 0.3)])
    sm.update([(1, 0.55, 0.28)])
    assert len(sm.objects) == 1

    # Wait for loss tolerance to expire
    time.sleep(0.6)

    # Call update with empty detections to trigger pruning
    sm.update([])
    assert len(sm.objects) == 0


def test_velocity_compensated_pruning():
    """Test that objects move backward with robot speed."""
    config = {
        'confirmation_threshold_frames': 1,
        'frame_loss_tolerance_sec': 2.0,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    # Add object at y = 0.3m
    sm.update([(1, 0.0, 0.3)])
    assert sm.objects[0].local_y == 0.3

    # Move robot forward at 0.1 m/s for 0.05s → object moves backward 0.005m
    sm.update([], robot_speed=0.1, dt=0.05)
    assert sm.objects[0].local_y == 0.295

    # Move robot forward 0.3m (3.0s at 0.1 m/s) → object should be pruned
    sm.update([], robot_speed=0.1, dt=3.0)
    assert len(sm.objects) == 0


def test_confirmation_threshold():
    """Test that objects only become confirmed after N frames."""
    config = {
        'confirmation_threshold_frames': 3,
        'frame_loss_tolerance_sec': 2.0,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    # First detection → confidence = 1
    sm.update([(1, 0.5, 0.3)])
    confirmed = sm.get_confirmed_objects()
    assert len(confirmed) == 0  # Not confirmed yet

    # Second detection → confidence = 2
    sm.update([(1, 0.55, 0.28)])
    confirmed = sm.get_confirmed_objects()
    assert len(confirmed) == 0  # Still not confirmed

    # Third detection → confidence = 3 → confirmed!
    sm.update([(1, 0.52, 0.31)])
    confirmed = sm.get_confirmed_objects()
    assert len(confirmed) == 1


def test_color_filtering():
    """Test that only same-color objects are merged."""
    config = {
        'confirmation_threshold_frames': 1,
        'frame_loss_tolerance_sec': 1.0,
        'tolerance_radius_m': 0.20
    }
    sm = SpatialMap(config)

    # Add RED (ID 1)
    sm.update([(1, 0.5, 0.3)])
    # Add GREEN (ID 2) at the same position
    sm.update([(2, 0.52, 0.31)])

    # Should be two separate objects (different colors)
    assert len(sm.objects) == 2

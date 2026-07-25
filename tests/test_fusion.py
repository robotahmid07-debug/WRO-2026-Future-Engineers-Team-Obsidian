import pytest
from src.spatial_map import SpatialMap

def test_spatial_gating():
    map = SpatialMap({'tolerance_radius_m': 0.2, 'confirmation_threshold_frames': 2, 'frame_loss_tolerance_sec': 1.0})
    # Add detection
    map.update([(1, 0.5, 0.3)])
    assert len(map.objects) == 1
    assert map.objects[0].confidence == 1
    # Second detection within tolerance -> merge
    map.update([(1, 0.55, 0.28)])
    assert len(map.objects) == 1
    assert map.objects[0].confidence == 2
    # Check confirmed
    confirmed = map.get_confirmed_objects()
    assert len(confirmed) == 1

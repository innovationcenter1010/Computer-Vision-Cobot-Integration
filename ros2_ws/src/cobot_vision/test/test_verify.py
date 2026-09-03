"""Post-place depth verdict (verify.py) — pure, no ROS."""

from cobot_vision.verify import classify_presence

CAN_HEIGHT = 87.0
TOLERANCE = 40.0


def test_unchanged_surface_is_still_present():
    present, delta = classify_presence(1900.0, 1903.0, TOLERANCE)
    assert present and delta == 3.0


def test_can_removed_reads_one_can_deeper():
    present, delta = classify_presence(1900.0, 1900.0 + CAN_HEIGHT, TOLERANCE)
    assert not present and delta == CAN_HEIGHT


def test_taller_object_placed_is_not_the_same_can():
    present, delta = classify_presence(1900.0, 1850.0, TOLERANCE)
    assert not present and delta == -50.0


def test_tolerance_boundary_is_inclusive():
    assert classify_presence(1900.0, 1940.0, TOLERANCE)[0]
    assert not classify_presence(1900.0, 1940.1, TOLERANCE)[0]

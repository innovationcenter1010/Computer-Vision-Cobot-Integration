import math

def _vector_from_points(start_point, end_point):
    return (
        float(end_point[0]) - float(start_point[0]),
        float(end_point[1]) - float(start_point[1]),
    )

def _dot(vector_a, vector_b):
    return (
        vector_a[0] * vector_b[0]
        + vector_a[1] * vector_b[1]
    )

def _cross_2d(vector_a, vector_b):
    return (
        vector_a[0] * vector_b[1]
        - vector_a[1] * vector_b[0]
    )

def _is_zero_vector(vector):
    return abs(vector[0]) < 1e-9 and abs(vector[1]) < 1e-9


def calculate_bin_rz(
    real_TL,
    real_BL,
    real_world_corners_mm,
):
    
    bin_top_left = real_world_corners_mm[0]
    bin_bottom_left = real_world_corners_mm[3]

    bin_left_edge_vector = _vector_from_points(
        bin_top_left,
        bin_bottom_left,
    )

    can_left_edge_vector = _vector_from_points(
        real_TL,
        real_BL,
    )

    if _is_zero_vector(bin_left_edge_vector):
        raise ValueError("Bin left-edge vector has zero length.")

    if _is_zero_vector(can_left_edge_vector):
        raise ValueError("Can left-edge vector has zero length.")

    dot_value = _dot(
        bin_left_edge_vector,
        can_left_edge_vector,
    )

    cross_value = _cross_2d(
        bin_left_edge_vector,
        can_left_edge_vector,
    )

    angle_rad = math.atan2(
        cross_value,
        dot_value,
    )

    angle_deg = math.degrees(angle_rad)

    return angle_deg


def calculate_real_rz(
    real_TL,
    real_BL,):


    can_left_edge_vector = _vector_from_points(
        real_TL,
        real_BL,
    )

    if _is_zero_vector(can_left_edge_vector):
        raise ValueError("Can left-edge vector has zero length.")

    # World +Y axis
    y_axis_vector = (0.0, 1.0)

    dot_value = _dot(
        y_axis_vector,
        can_left_edge_vector,
    )

    cross_value = _cross_2d(
        y_axis_vector,
        can_left_edge_vector,
    )

    angle_rad = math.atan2(
        cross_value,
        dot_value,
    )

    angle_deg = math.degrees(angle_rad)

    return angle_deg
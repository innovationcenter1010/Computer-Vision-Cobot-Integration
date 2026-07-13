import math


def _subtract_points(point_a, point_b):
    return (
        float(point_a[0]) - float(point_b[0]),
        float(point_a[1]) - float(point_b[1]),
    )


def _length(vector):
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2)


def project_real_point_to_bin_axes(real_x, real_y, real_world_corners_mm):
    """
    Converts a real-world point into bin-local coordinates.

    real_world_corners_mm must be ordered:
        [0] = top-left
        [1] = top-right
        [2] = bottom-right
        [3] = bottom-left

    Returns:
        across_mm:
            Distance from top-left toward top-right.

        down_mm:
            Distance from top-left toward bottom-left.

    This uses a 2D basis solve instead of simple dot products, so it works
    better if the bin is rotated in robot coordinates.
    """

    top_left = real_world_corners_mm[0]
    top_right = real_world_corners_mm[1]
    bottom_left = real_world_corners_mm[3]

    right_vector = _subtract_points(top_right, top_left)
    down_vector = _subtract_points(bottom_left, top_left)

    point_vector = _subtract_points(
        (real_x, real_y),
        top_left,
    )

    rx, ry = right_vector
    dx, dy = down_vector
    px, py = point_vector

    determinant = rx * dy - ry * dx

    if abs(determinant) < 1e-9:
        raise ValueError(
            "Invalid real_world_corners_mm: top-left, top-right, and bottom-left "
            "do not form a usable bin coordinate system."
        )

    # Solve:
    # point = top_left + u * right_vector + v * down_vector
    u = (px * dy - py * dx) / determinant
    v = (rx * py - ry * px) / determinant

    bin_width_mm = _length(right_vector)
    bin_height_mm = _length(down_vector)

    across_mm = u * bin_width_mm
    down_mm = v * bin_height_mm

    return across_mm, down_mm


def order_obb_detections_by_real_position(
    detections,
    real_world_corners_mm,
    row_tolerance_mm,
):
    """
    Orders YOLO OBB detections in a snake pattern using real-world bin coordinates.

    The path starts closest to the real-world top-left bin corner:
        row 1: left  -> right
        row 2: right -> left
        row 3: left  -> right

    Returns:
        The same OBBDetection objects, reordered.
    """

    if len(detections) == 0:
        return []

    projected = []

    for detection in detections:
        if detection.real_x is None or detection.real_y is None:
            raise ValueError(
                "Detection must have real_x and real_y before pathing. "
                "Apply homography before calling order_obb_detections_by_real_position()."
            )

        across_mm, down_mm = project_real_point_to_bin_axes(
            detection.real_x,
            detection.real_y,
            real_world_corners_mm,
        )

        # These are only for debugging. They do not affect robot movement.
        detection.path_across_mm = across_mm
        detection.path_down_mm = down_mm

        projected.append(
            {
                "detection": detection,
                "across_mm": across_mm,
                "down_mm": down_mm,
            }
        )

    # Sort from top of bin to bottom of bin.
    projected.sort(key=lambda item: item["down_mm"])

    rows = []
    current_row = [projected[0]]

    for item in projected[1:]:
        current_row_down_average = (
            sum(row_item["down_mm"] for row_item in current_row)
            / len(current_row)
        )

        if abs(item["down_mm"] - current_row_down_average) <= row_tolerance_mm:
            current_row.append(item)
        else:
            rows.append(current_row)
            current_row = [item]

    rows.append(current_row)

    ordered_detections = []

    for row_index, row in enumerate(rows):
        if row_index % 2 == 0:
            # First row starts closest to top-left.
            row_sorted = sorted(row, key=lambda item: item["across_mm"])
        else:
            # Next row snakes back.
            row_sorted = sorted(
                row,
                key=lambda item: item["across_mm"],
                reverse=True,
            )

        ordered_detections.extend(
            item["detection"]
            for item in row_sorted
        )

    return ordered_detections


def print_obb_order(detections, real_world_corners_mm):
    """
    Prints ordered OBBDetection objects using bin-local pathing coordinates.
    """

    print("Ordered OBB detections:")

    for i, detection in enumerate(detections):
        across_mm, down_mm = project_real_point_to_bin_axes(
            detection.real_x,
            detection.real_y,
            real_world_corners_mm,
        )

        print(
            f"{i + 1}: "
            f"real=({detection.real_x:.2f}, {detection.real_y:.2f}), "
            f"path_across={across_mm:.2f}, "
            f"path_down={down_mm:.2f}, "
            f"confidence={detection.confidence:.3f}",
            flush=True,
        )
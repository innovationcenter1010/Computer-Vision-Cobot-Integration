"""Snake pick ordering in the ROBOT base frame — no bin geometry.

Detections carry real_x / real_y (mm, robot base) from the camera->robot
calibration. Ordering groups them into rows along one robot axis and
serpentines along the other, so the arm finishes a row and snakes back on the
next. This deliberately uses the robot axes directly instead of a bin-local
frame: bin-corner coordinates go stale the moment the bin shifts, and the
pick order does not need bin alignment — only a consistent traversal.
"""


def _axis_getters(row_axis):
    """(row_value, across_value) accessors for the chosen row axis.

    row_axis groups rows (near-constant value per row); the other axis is the
    serpentine direction within a row.
    """
    if row_axis == "y":
        return (lambda d: d.real_y), (lambda d: d.real_x)
    if row_axis == "x":
        return (lambda d: d.real_x), (lambda d: d.real_y)
    raise ValueError(f"pathing_row_axis must be 'x' or 'y', got {row_axis!r}")


def order_detections_snake(detections, row_axis="y", row_tolerance_mm=50.0):
    """Order detections in a robot-frame snake pattern.

    - rows: bands of near-constant position along `row_axis` (within
      row_tolerance_mm), traversed from the smallest row value upward
    - within a row: sorted along the other axis, alternating direction each
      row so the path snakes instead of jumping back

    Returns the same detection objects, reordered. Each detection's
    path_down_mm / path_across_mm are set for debugging (they do not affect
    robot movement).
    """
    if len(detections) == 0:
        return []

    row_value, across_value = _axis_getters(row_axis)

    for detection in detections:
        if detection.real_x is None or detection.real_y is None:
            raise ValueError(
                "Detection must have real_x and real_y before pathing. Apply "
                "the camera->robot calibration transform before ordering."
            )
        detection.path_down_mm = row_value(detection)
        detection.path_across_mm = across_value(detection)

    ordered_by_row = sorted(detections, key=row_value)

    rows = []
    current_row = [ordered_by_row[0]]
    for detection in ordered_by_row[1:]:
        current_row_average = (
            sum(row_value(d) for d in current_row) / len(current_row)
        )
        if abs(row_value(detection) - current_row_average) <= row_tolerance_mm:
            current_row.append(detection)
        else:
            rows.append(current_row)
            current_row = [detection]
    rows.append(current_row)

    ordered = []
    for row_index, row in enumerate(rows):
        snake_back = row_index % 2 == 1
        ordered.extend(sorted(row, key=across_value, reverse=snake_back))

    return ordered

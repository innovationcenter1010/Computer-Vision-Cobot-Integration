from pathlib import Path
from layers import Layer


def create_logical_path(layer):
    """
    Creates a logical path through all detected cans.

    The path is organized:
    1. Top row to bottom row
    2. Left-to-right on the first row
    3. Right-to-left on the second row
    4. Repeats in a snake pattern

    Returns an array of coordinate pairs:
    [(x1, y1), (x2, y2), ...]
    """

    cans = layer.cans

    if len(cans) == 0:
        return []

    # Sort all cans by vertical position first
    cans_sorted = sorted(cans, key=lambda can: can.center_y)

    rows = []
    current_row = [cans_sorted[0]]

    # Use average can height to decide what counts as the same row
    average_height = sum(can.height for can in cans_sorted) / len(cans_sorted)
    row_tolerance = average_height / 2

    # Group cans into rows
    for can in cans_sorted[1:]:
        row_center_y = sum(c.center_y for c in current_row) / len(current_row)

        if abs(can.center_y - row_center_y) <= row_tolerance:
            current_row.append(can)
        else:
            rows.append(current_row)
            current_row = [can]

    rows.append(current_row)

    path = []

    # Sort each row using a snake pattern
    for row_index, row in enumerate(rows):
        if row_index % 2 == 0:
            # First row, third row, etc: left to right
            row_sorted = sorted(row, key=lambda can: can.center_x)
        else:
            # Second row, fourth row, etc: right to left
            row_sorted = sorted(row, key=lambda can: can.center_x, reverse=True)

        for can in row_sorted:
            path.append((can.center_x, can.center_y))

    return path


def print_path(path):
    """
    Prints the path clearly.
    """

    print("Logical path coordinates:")

    for i, coordinate in enumerate(path):
        print(f"{i + 1}: {coordinate}")

    print("\nArray output:")
    print(path)


if __name__ == "__main__":
    # pathing.py is inside src/app
    # sample_data.JSON is inside src
    json_file_path = Path(__file__).resolve().parent / "sample_data.JSON"

    layer = Layer(json_file_path)

    path = create_logical_path(layer)

    print_path(path)
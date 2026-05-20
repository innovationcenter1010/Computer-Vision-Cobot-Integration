from pathlib import Path
from layers import Layer


def order_cans(cans):
    """
    Takes a list of Can objects and returns the Can objects
    in a logical snake-pattern order.

    This returns Can objects, NOT coordinates.
    """

    if len(cans) == 0:
        return []

    # Sort all cans from top to bottom
    cans_sorted = sorted(cans, key=lambda can: can.center_y)

    # Use average can height to decide which cans are in the same row
    average_height = sum(can.height for can in cans_sorted) / len(cans_sorted)
    row_tolerance = average_height / 2

    rows = []
    current_row = [cans_sorted[0]]

    # Group cans into rows
    for can in cans_sorted[1:]:
        current_row_center_y = sum(c.center_y for c in current_row) / len(current_row)

        if abs(can.center_y - current_row_center_y) <= row_tolerance:
            current_row.append(can)
        else:
            rows.append(current_row)
            current_row = [can]

    rows.append(current_row)

    ordered_cans = []

    # Snake pattern:
    # row 0: left to right
    # row 1: right to left
    # row 2: left to right
    # etc.
    for row_index, row in enumerate(rows):
        if row_index % 2 == 0:
            row_sorted = sorted(row, key=lambda can: can.center_x)
        else:
            row_sorted = sorted(row, key=lambda can: can.center_x, reverse=True)

        ordered_cans.extend(row_sorted)

    return ordered_cans


def order_layer(layer):
    """
    Adds an ordered list of Can objects to the Layer object.

    After this runs:

    layer.order

    will be a list of ordered Can objects.
    """

    ordered_cans = order_cans(layer.cans)

    # Store the ordered Can objects
    layer.order = ordered_cans

    # Optional: also update layer.cans so the original cans list is ordered too
    layer.cans = ordered_cans

    layer.size = len(layer.cans)

    return layer


class OrderedLayer(Layer):
    """
    A Layer object that automatically orders its Can objects.

    In main.py, you can do:

    layers = OrderedLayer("sample_data.JSON")
    print(layers.order)

    layers.order is a list of Can objects.
    """

    def __init__(self, json_file_path):
        super().__init__(json_file_path)
        order_layer(self)


def print_order(layer):
    """
    Prints the ordered Can objects using their center coordinates.
    """

    print("Ordered Can objects:")

    for i, can in enumerate(layer.order):
        print(
            f"{i + 1}: "
            f"class={can.can_class}, "
            f"center=({can.center_x}, {can.center_y}), "
            f"confidence={can.confidence}"
        )


if __name__ == "__main__":
    # pathing.py and sample_data.JSON are both inside src/app
    json_file_path = Path(__file__).resolve().parent / "sample_data.JSON"

    layers = OrderedLayer(json_file_path)

    print_order(layers)
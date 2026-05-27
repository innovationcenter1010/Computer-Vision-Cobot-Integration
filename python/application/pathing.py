def order_cans(cans):
    """
    Takes a list of Can objects and returns them in snake-pattern order.
    """

    if len(cans) == 0:
        return []

    cans_sorted = sorted(cans, key=lambda can: can.center_y)

    average_height = sum(can.height for can in cans_sorted) / len(cans_sorted)
    row_tolerance = average_height / 2

    rows = []
    current_row = [cans_sorted[0]]

    for can in cans_sorted[1:]:
        current_row_center_y = sum(c.center_y for c in current_row) / len(current_row)

        if abs(can.center_y - current_row_center_y) <= row_tolerance:
            current_row.append(can)
        else:
            rows.append(current_row)
            current_row = [can]

    rows.append(current_row)

    ordered_cans = []

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

    After this runs, layer.order is a list of ordered Can objects.
    """

    ordered_cans = order_cans(layer.cans)

    layer.order = ordered_cans
    layer.cans = ordered_cans
    layer.size = len(layer.cans)

    return layer


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
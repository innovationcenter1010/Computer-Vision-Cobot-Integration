from pathlib import Path
from layers import OrderedLayer
from config import BIN_IMAGE_CORNERS, BIN_REAL_WORLD_CORNERS

JSON_FILE_NAME = "sample_data.JSON"

if __name__ == "__main__":
    current_folder = Path(__file__).resolve().parent
    json_file_path = current_folder / JSON_FILE_NAME

    # One line loads, orders, and translates everything
    layer = OrderedLayer(json_file_path, BIN_IMAGE_CORNERS, BIN_REAL_WORLD_CORNERS)

    print(f"Number of cans detected: {layer.size}")
    print("\nOrdered cans with real-world coordinates:")

    for i, can in enumerate(layer.order):
        print(
            f"{i + 1}: "
            f"image=({can.center_x}, {can.center_y}) "
            f"real=({can.real_x:.2f}, {can.real_y:.2f})"
        )
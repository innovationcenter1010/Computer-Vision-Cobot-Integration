import config
from pathlib import Path
from bin import Bin

JSON_FILE_NAME = "sample_data.JSON"

if __name__ == "__main__":
    current_folder = Path(__file__).resolve().parent
    json_file_path = current_folder / JSON_FILE_NAME

    bin = Bin()
    bin.add_layer(json_file_path)

    print(f"Number of cans detected: {bin.layer.size}")
    print("\nOrdered cans with real-world coordinates:")

    for i, can in enumerate(bin.layer.order):
        print(
            f"{i + 1}: "
            f"image=({can.center_x}, {can.center_y}) "
            f"real=({can.real_x:.2f}, {can.real_y:.2f}, {can.real_z:.2f})"
        )
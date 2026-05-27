import json
from pathlib import Path
from bin import Bin


TEST_JSON_FOLDER_NAME = "Test_json"

# If a test JSON file is missing or empty, this script will generate it.
# Existing non-empty files will be left alone.
GENERATE_MISSING_OR_EMPTY_JSON_FILES = True


TEST_FILES = {
    "01_new_bin_layer1_start20.json": 20,
    "02_after_layer1_too_small_3.json": 3,
    "03_after_layer1_next_layer_16.json": 16,
    "04_after_layer2_next_layer_12.json": 12,
    "05_after_layer3_next_layer_9.json": 9,
    "06_after_layer4_next_layer_7.json": 7,
    "07_after_layer5_next_layer_5.json": 5,
    "08_after_layer6_next_layer_4.json": 4,
    "09_after_layer7_not_empty_2.json": 2,
    "10_after_layer7_empty_0.json": 0,
    "11_new_bin_layer1_start_small_6.json": 6,
    "12_new_bin_after_layer1_too_small_3.json": 3,
    "13_new_bin_after_layer1_next_layer_5.json": 5,
}


def make_fake_predictions(count):
    """
    Creates fake Roboflow-style can detections.
    Each detected can becomes one prediction object.
    """
    predictions = []

    start_x = 130
    start_y = 90
    spacing_x = 55
    spacing_y = 55
    cans_per_row = 5

    for i in range(count):
        row = i // cans_per_row
        col = i % cans_per_row

        predictions.append({
            "width": 40,
            "height": 45,
            "x": start_x + col * spacing_x,
            "y": start_y + row * spacing_y,
            "confidence": 0.95,
            "class_id": 0,
            "class": "top can"
        })

    return [
        {
            "predictions": {
                "image": {
                    "width": 640,
                    "height": 480
                },
                "predictions": predictions
            }
        }
    ]


def write_test_json(path, can_count):
    with open(path, "w") as file:
        json.dump(make_fake_predictions(can_count), file, indent=2)


def prepare_test_json_files(folder):
    """
    Uses JSON files from python/application/Test_json.

    If a file is missing or empty, this function creates it.
    Existing non-empty JSON files are not overwritten.
    """
    folder.mkdir(exist_ok=True)

    paths = {}

    for file_name, can_count in TEST_FILES.items():
        path = folder / file_name

        if not path.exists() or path.stat().st_size == 0:
            if GENERATE_MISSING_OR_EMPTY_JSON_FILES:
                write_test_json(path, can_count)
            else:
                raise FileNotFoundError(f"Missing or empty test JSON file: {path}")

        paths[file_name] = path

    return paths


def pick_everything(current_bin):
    """
    Simulates the cobot picking every can in the current layer.
    """
    picked = 0

    while True:
        can = current_bin.pickup()

        if can is None:
            break

        picked += 1

    print(f"  picked {picked} cans")
    return picked


def check_status(actual_status, expected_status, test_name):
    print(f"\n{test_name}")
    print(f"  expected: {expected_status}")
    print(f"  actual:   {actual_status}")

    if actual_status != expected_status:
        raise AssertionError(
            f"{test_name} failed. Expected {expected_status}, got {actual_status}"
        )

    print("  PASSED")


def check_bin_api():
    """
    Makes sure bin.py has the updated methods/constants required by this test.
    """
    required_attributes = [
        "NEXT_LAYER_READY",
        "CONTINUE_CURRENT_LAYER",
        "BIN_COMPLETE",
        "add_layer",
        "check_layer_complete",
        "pickup",
    ]

    for attribute in required_attributes:
        if not hasattr(Bin, attribute):
            raise AttributeError(
                f"Bin is missing '{attribute}'. "
                f"Make sure bin.py has the updated layer-completion logic."
            )


if __name__ == "__main__":
    print("=== sample_main_logic_test.py started ===", flush=True)

    check_bin_api()

    current_folder = Path(__file__).resolve().parent
    test_json_folder = current_folder / TEST_JSON_FOLDER_NAME

    print(f"\nUsing test JSON folder:")
    print(f"  {test_json_folder}")

    paths = prepare_test_json_files(test_json_folder)

    print("\n=== TEST JSON FILES READY ===")
    for name, path in paths.items():
        print(f"  {name}: {path.stat().st_size} bytes")

    print("\n=== START BIN 1 ===")

    current_bin = Bin()

    # New bin layer 1 is accepted immediately.
    current_bin.add_layer(paths["01_new_bin_layer1_start20.json"])

    print("\nLoaded bin 1, layer 1")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    print(f"  cans detected: {current_bin.layer.size}")

    pick_everything(current_bin)

    # Layer 1 had 20 cans.
    # Threshold = 20 * 0.7 = 14.
    # 3 cans is too small, so stay on current layer.
    status = current_bin.check_layer_complete(
        paths["02_after_layer1_too_small_3.json"]
    )
    check_status(
        status,
        Bin.CONTINUE_CURRENT_LAYER,
        "Test 1: too few cans after layer 1, stay on current layer"
    )

    pick_everything(current_bin)

    # 16 cans is enough to count as the next layer.
    status = current_bin.check_layer_complete(
        paths["03_after_layer1_next_layer_16.json"]
    )
    check_status(
        status,
        Bin.NEXT_LAYER_READY,
        "Test 2: enough cans detected, advance to layer 2"
    )

    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")

    pick_everything(current_bin)

    status = current_bin.check_layer_complete(
        paths["04_after_layer2_next_layer_12.json"]
    )
    check_status(status, Bin.NEXT_LAYER_READY, "Test 3: advance to layer 3")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    pick_everything(current_bin)

    status = current_bin.check_layer_complete(
        paths["05_after_layer3_next_layer_9.json"]
    )
    check_status(status, Bin.NEXT_LAYER_READY, "Test 4: advance to layer 4")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    pick_everything(current_bin)

    status = current_bin.check_layer_complete(
        paths["06_after_layer4_next_layer_7.json"]
    )
    check_status(status, Bin.NEXT_LAYER_READY, "Test 5: advance to layer 5")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    pick_everything(current_bin)

    status = current_bin.check_layer_complete(
        paths["07_after_layer5_next_layer_5.json"]
    )
    check_status(status, Bin.NEXT_LAYER_READY, "Test 6: advance to layer 6")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    pick_everything(current_bin)

    status = current_bin.check_layer_complete(
        paths["08_after_layer6_next_layer_4.json"]
    )
    check_status(status, Bin.NEXT_LAYER_READY, "Test 7: advance to layer 7")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    pick_everything(current_bin)

    # Layer 7 still has cans, so the bin should not complete.
    status = current_bin.check_layer_complete(
        paths["09_after_layer7_not_empty_2.json"]
    )
    check_status(
        status,
        Bin.CONTINUE_CURRENT_LAYER,
        "Test 8: layer 7 still has cans, do not complete bin"
    )

    pick_everything(current_bin)

    # Layer 7 detects 0 cans, so bin is complete.
    status = current_bin.check_layer_complete(
        paths["10_after_layer7_empty_0.json"]
    )
    check_status(
        status,
        Bin.BIN_COMPLETE,
        "Test 9: layer 7 has 0 cans, bin is complete"
    )

    print("\n=== CREATE NEW BIN OBJECT ===")

    current_bin = Bin()

    current_bin.add_layer(paths["11_new_bin_layer1_start_small_6.json"])

    print("\nLoaded bin 2, layer 1")
    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")
    print(f"  cans detected: {current_bin.layer.size}")

    pick_everything(current_bin)

    # New bin layer 1 had 6 cans.
    # Threshold = 6 * 0.7 = 4.2.
    # 3 cans is too small, so stay on layer 1.
    status = current_bin.check_layer_complete(
        paths["12_new_bin_after_layer1_too_small_3.json"]
    )
    check_status(
        status,
        Bin.CONTINUE_CURRENT_LAYER,
        "Test 10: new bin layer 1 too small, stay on current layer"
    )

    pick_everything(current_bin)

    # 5 cans is enough compared to 6 * 0.7 = 4.2.
    status = current_bin.check_layer_complete(
        paths["13_new_bin_after_layer1_next_layer_5.json"]
    )
    check_status(
        status,
        Bin.NEXT_LAYER_READY,
        "Test 11: new bin advances from layer 1 to layer 2"
    )

    print(f"  depth index: {current_bin.depth_index}")
    print(f"  z-depth: {current_bin.depth}")

    print("\nALL BIN LOGIC TESTS PASSED")
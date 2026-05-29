from ultralytics import YOLO
import torch

def main():
    print("=== ULTRALYTICS SMOKE TEST START ===")

    # 1. Check PyTorch + CUDA
    print(f"PyTorch version: {torch.__version__}")
    print(f"CUDA available: {torch.cuda.is_available()}")

    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)}")

    # 2. Load YOLO model (this triggers download if not cached)
    print("\nLoading YOLO model...")
    model = YOLO("yolov8n.pt")  # smallest model, fastest test

    print("Model loaded successfully!")

    # 3. Run inference on a test image URL (Ultralytics provides a sample image)
    print("\nRunning inference...")

    results = model("https://ultralytics.com/images/bus.jpg")

    # 4. Print results summary
    for r in results:
        print("\n--- DETECTIONS ---")
        print(r.boxes)

    print("\n=== ULTRALYTICS SMOKE TEST COMPLETE ===")


if __name__ == "__main__":
    main()
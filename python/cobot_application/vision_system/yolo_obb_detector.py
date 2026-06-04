from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np
from ultralytics import YOLO


@dataclass
class OBBDetection:
    class_id: int
    confidence: float
    pixel_corners: list
    normalized_corners: list
    center_x: float
    center_y: float


class YoloOBBDetector:
    def __init__(self, model_path: str | Path, conf: float = 0.25, imgsz: int = 640):
        self.model_path = Path(model_path)

        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO model not found: {self.model_path}. "
                f"Train the model first."
            )

        self.conf = conf
        self.imgsz = imgsz
        self.model = YOLO(str(self.model_path))

    def detect(self, image_path: str | Path) -> list[OBBDetection]:
        image_path = Path(image_path)

        if not image_path.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")

        results = self.model.predict(
            source=str(image_path),
            imgsz=self.imgsz,
            conf=self.conf,
            save=False,
            verbose=False,
        )

        detections: list[OBBDetection] = []

        for result in results:
            if result.obb is None or len(result.obb) == 0:
                continue

            image_height, image_width = result.orig_shape

            pixel_corners = result.obb.xyxyxyxy.cpu().numpy()
            classes = result.obb.cls.cpu().numpy()
            confidences = result.obb.conf.cpu().numpy()

            normalized_corners = pixel_corners.copy()
            normalized_corners[:, :, 0] /= image_width
            normalized_corners[:, :, 1] /= image_height

            for box, norm_box, cls, conf in zip(
                pixel_corners,
                normalized_corners,
                classes,
                confidences,
            ):
                box = np.array(box, dtype=float)

                detections.append(
                    OBBDetection(
                        class_id=int(cls),
                        confidence=float(conf),
                        pixel_corners=box.tolist(),
                        normalized_corners=norm_box.tolist(),
                        center_x=float(box[:, 0].mean()),
                        center_y=float(box[:, 1].mean()),
                    )
                )

        return detections

    @staticmethod
    def save_detections_json(detections: list[OBBDetection], output_path: str | Path):
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        data = [
            {
                "class_id": det.class_id,
                "confidence": det.confidence,
                "center_x": det.center_x,
                "center_y": det.center_y,
                "pixel_corners": det.pixel_corners,
                "normalized_corners": det.normalized_corners,
            }
            for det in detections
        ]

        output_path.write_text(json.dumps(data, indent=2))
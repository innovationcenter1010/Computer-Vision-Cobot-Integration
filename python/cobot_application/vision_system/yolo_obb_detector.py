from dataclasses import dataclass
from pathlib import Path
import json
import math

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

    real_x: float | None = None
    real_y: float | None = None
    real_z: float | None = None
    real_rz: float | None = None

    image_path: str | None = None


class YoloOBBDetector:
    def __init__(
        self,
        model_path: str | Path,
        conf: float = 0.85,
        imgsz: int = 640,
        iou: float = 0.4,
        max_det: int = 30,
    ):
        self.model_path = Path(model_path)

        if not self.model_path.exists():
            raise FileNotFoundError(f"YOLO OBB model not found: {self.model_path}")

        self.conf = conf
        self.imgsz = imgsz
        self.iou = iou
        self.max_det = max_det

        self.model = YOLO(str(self.model_path))

    def detect(
        self,
        image: np.array | Path,
        save_visual: bool = False,
        visual_project_dir: str | Path | None = None,
        visual_name: str | None = None,
    ) -> list[OBBDetection]:


        #------------ image path handling --------------
        if isinstance(image, (str, Path)):
            image_path = Path(image)

            if not image_path.exists():
                raise FileNotFoundError(f"Image not found: {image_path}")

            predict_args["source"] = str(image_path)

        #------------- numpy array handling -----------------------

        elif isinstance(image, np.ndarray):
            predict_args["source"] = image

        else:
            raise TypeError("Input must be image path or numpy array")


        #

        predict_args = {
            "imgsz": self.imgsz,
            "conf": self.conf,
            "iou": self.iou,
            "max_det": self.max_det,
            "save": save_visual,
            "verbose": False,
        }

        #add output directories
        if save_visual and visual_project_dir is not None:
            predict_args["project"] = str(visual_project_dir)
        if save_visual and visual_name is not None:
            predict_args["name"] = visual_name

        #actually run model
        results = self.model.predict(**predict_args)

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

                center_x = float(box[:, 0].mean())
                center_y = float(box[:, 1].mean())

                detections.append(
                    OBBDetection(
                        class_id=int(cls),
                        confidence=float(conf),
                        pixel_corners=box.tolist(),
                        normalized_corners=norm_box.tolist(),
                        center_x=center_x,
                        center_y=center_y,
                        image_path=str(image_path),
                    )
                )

        return detections

    @staticmethod
    def save_json(detections: list[OBBDetection], output_path: str | Path):
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
                "real_x": det.real_x,
                "real_y": det.real_y,
                "real_z": det.real_z,
                "real_rz": det.real_rz,
                "image_path": det.image_path,
            }
            for det in detections
        ]

        output_path.write_text(json.dumps(data, indent=2))


def remove_duplicate_detections(
    detections: list[OBBDetection],
    min_center_distance_px: float = 50.0,
) -> list[OBBDetection]:
    """
    Removes detections whose centers are too close together.
    Keeps the highest-confidence detection.
    """

    sorted_detections = sorted(
        detections,
        key=lambda det: det.confidence,
        reverse=True,
    )

    kept: list[OBBDetection] = []

    for det in sorted_detections:
        duplicate = False

        for kept_det in kept:
            dx = det.center_x - kept_det.center_x
            dy = det.center_y - kept_det.center_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < min_center_distance_px:
                duplicate = True
                break

        if not duplicate:
            kept.append(det)

    return kept
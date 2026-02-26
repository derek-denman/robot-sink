#!/usr/bin/env python3
"""TensorRT-backed detector node publishing Detection2DArray."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


def _to_list(value: Any) -> List[Any]:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        return list(value.tolist())
    return list(value)


def _decode_compressed(msg: CompressedImage) -> Any:
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    arr = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Failed to decode CompressedImage")
    return image


def _decode_image(msg: Image) -> Any:
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    encoding = str(msg.encoding).lower()
    h = int(msg.height)
    w = int(msg.width)
    if h <= 0 or w <= 0:
        raise RuntimeError("Invalid Image dimensions")

    if encoding in {"bgr8", "rgb8"}:
        channels = 3
    elif encoding in {"mono8", "8uc1"}:
        channels = 1
    else:
        raise RuntimeError(f"Unsupported encoding: {encoding}")

    min_step = w * channels
    row_step = max(int(msg.step), min_step)
    expected = h * row_step
    payload = bytes(msg.data)
    if len(payload) < expected:
        raise RuntimeError("Image payload smaller than expected")

    arr = np.frombuffer(payload[:expected], dtype=np.uint8).reshape((h, row_step))
    arr = arr[:, :min_step]

    if channels == 1:
        mono = arr.reshape((h, w))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    color = arr.reshape((h, w, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    return color


class TrtDetectorNode(Node):
    """Subscribe image topic, run YOLO TensorRT inference, publish Detection2DArray."""

    def __init__(self) -> None:
        super().__init__("toy_shoe_trt_detector")

        self.declare_parameter("model_engine_path", "")
        self.declare_parameter("image_topic", "/oak/rgb/image_raw/compressed")
        self.declare_parameter("input_compressed", True)
        self.declare_parameter("detections_topic", "/perception/detections_2d")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_det", 100)
        self.declare_parameter("device", "")
        self.declare_parameter("frame_id_override", "")

        self._engine_path = Path(str(self.get_parameter("model_engine_path").value)).expanduser()
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._input_compressed = bool(self.get_parameter("input_compressed").value)
        self._detections_topic = str(self.get_parameter("detections_topic").value)
        self._conf = float(self.get_parameter("confidence_threshold").value)
        self._iou = float(self.get_parameter("iou_threshold").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._max_det = int(self.get_parameter("max_det").value)
        self._device = str(self.get_parameter("device").value)
        self._frame_id_override = str(self.get_parameter("frame_id_override").value)

        self._model = None
        self._class_names: Dict[int, str] = {}
        self._load_model()

        self._pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)
        if self._input_compressed:
            self._sub = self.create_subscription(
                CompressedImage,
                self._image_topic,
                self._on_compressed,
                qos_profile_sensor_data,
            )
        else:
            self._sub = self.create_subscription(
                Image,
                self._image_topic,
                self._on_image,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            "trt_detector_node started "
            f"image_topic={self._image_topic} compressed={self._input_compressed} "
            f"detections_topic={self._detections_topic}"
        )

    def _load_model(self) -> None:
        if not self._engine_path:
            self.get_logger().warn("No model_engine_path configured; publishing empty detections.")
            return

        if not self._engine_path.exists():
            self.get_logger().warn(f"Engine file not found: {self._engine_path}; publishing empty detections.")
            return

        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            self.get_logger().error("Ultralytics unavailable; detector will publish empty detections")
            return

        self._model = YOLO(str(self._engine_path))
        names = getattr(self._model, "names", {})
        if isinstance(names, dict):
            self._class_names = {int(k): str(v) for k, v in names.items()}
        elif isinstance(names, list):
            self._class_names = {idx: str(name) for idx, name in enumerate(names)}
        else:
            self._class_names = {0: "toy", 1: "shoe"}

        self.get_logger().info(f"Loaded TensorRT engine: {self._engine_path}")

    def _infer(self, image_bgr: Any) -> List[Tuple[float, float, float, float, float, int]]:
        if self._model is None:
            return []

        kwargs: Dict[str, Any] = {
            "source": image_bgr,
            "conf": self._conf,
            "iou": self._iou,
            "imgsz": self._imgsz,
            "max_det": self._max_det,
            "verbose": False,
        }
        if self._device:
            kwargs["device"] = self._device

        results = self._model.predict(**kwargs)
        if not results:
            return []

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []

        xyxy = _to_list(boxes.xyxy)
        confs = _to_list(boxes.conf)
        classes = _to_list(boxes.cls)

        out: List[Tuple[float, float, float, float, float, int]] = []
        for coords, conf, cls in zip(xyxy, confs, classes):
            x1, y1, x2, y2 = [float(v) for v in coords]
            out.append((x1, y1, x2, y2, float(conf), int(cls)))
        return out

    def _make_detection_msg(
        self,
        header: Any,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        score: float,
        class_idx: int,
    ) -> Detection2D:
        det = Detection2D()
        det.header = header
        class_name = self._class_names.get(class_idx, str(class_idx))

        bbox = BoundingBox2D()
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)

        # vision_msgs/BoundingBox2D center is Pose2D on Humble.
        if hasattr(bbox.center, "x"):
            bbox.center.x = float(cx)
            bbox.center.y = float(cy)
            bbox.center.theta = 0.0
        else:
            bbox.center.position.x = float(cx)
            bbox.center.position.y = float(cy)
            bbox.center.position.z = 0.0

        bbox.size_x = float(w)
        bbox.size_y = float(h)
        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        if hasattr(hyp, "hypothesis"):
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(score)
        else:
            hyp.id = class_name
            hyp.score = float(score)
        det.results.append(hyp)
        if hasattr(det, "id"):
            det.id = class_name
        return det

    def _publish_detections(self, source_header: Any, detections: List[Tuple[float, float, float, float, float, int]]) -> None:
        msg = Detection2DArray()
        msg.header = source_header
        if self._frame_id_override:
            msg.header.frame_id = self._frame_id_override

        for x1, y1, x2, y2, score, class_idx in detections:
            det = self._make_detection_msg(
                header=msg.header,
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                score=score,
                class_idx=class_idx,
            )
            msg.detections.append(det)

        self._pub.publish(msg)

    def _on_compressed(self, msg: CompressedImage) -> None:
        try:
            image = _decode_compressed(msg)
            detections = self._infer(image)
            self._publish_detections(msg.header, detections)
        except Exception as exc:  # pragma: no cover - runtime path
            self.get_logger().warn(f"Compressed image inference failed: {exc}")

    def _on_image(self, msg: Image) -> None:
        try:
            image = _decode_image(msg)
            detections = self._infer(image)
            self._publish_detections(msg.header, detections)
        except Exception as exc:  # pragma: no cover - runtime path
            self.get_logger().warn(f"Raw image inference failed: {exc}")


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrtDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

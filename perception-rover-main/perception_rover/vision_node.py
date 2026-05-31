import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
from ament_index_python.packages import get_package_share_directory


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.bbox_pub = self.create_publisher(String, '/bbox', 10)
        self.waste_pub = self.create_publisher(String, '/waste_bbox', 10)

        self._model = None
        self._load_model()
        self.get_logger().info('Vision Node started')

    def _load_model(self):
        model_path = os.path.join(
            get_package_share_directory('perception_rover'),
            'models', 'yolov5n.onnx')
        if os.path.exists(model_path):
            self._model = cv2.dnn.readNet(model_path)
            self._model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info(f'YOLOv5 ONNX loaded from {model_path}')
        else:
            self.get_logger().warn(
                f'Model not found at {model_path} — using HSV fallback')

    def _yolo_infer(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame_bgr, 2/255.0, (640, 640), swapRB=True, crop=False)
        self._model.setInput(blob)
        outputs = self._model.forward()

        output = outputs[0]
        boxes_raw = output[:, :4]
        objectness = output[:, 4]
        class_scores = output[:, 5:]

        confidence = objectness * np.max(class_scores, axis=1)
        mask = confidence > 0.4
        if not np.any(mask):
            return []

        boxes_raw = boxes_raw[mask]
        confidence = confidence[mask]
        class_ids = np.argmax(class_scores[mask], axis=1)

        boxes_xyxy = np.zeros_like(boxes_raw)
        boxes_xyxy[:, 0] = (boxes_raw[:, 0] - boxes_raw[:, 2] / 2) * w / 640
        boxes_xyxy[:, 1] = (boxes_raw[:, 1] - boxes_raw[:, 3] / 2) * h / 640
        boxes_xyxy[:, 2] = (boxes_raw[:, 0] + boxes_raw[:, 2] / 2) * w / 640
        boxes_xyxy[:, 3] = (boxes_raw[:, 1] + boxes_raw[:, 3] / 2) * h / 640

        boxes_xyxy = np.clip(boxes_xyxy, 0, max(w, h))

        indices = cv2.dnn.NMSBoxes(
            boxes_xyxy.astype(np.int32).tolist(),
            confidence.tolist(), 0.4, 0.45)
        if len(indices) == 0:
            return []

        results = []
        for i in indices.flatten():
            x1, y1, x2, y2 = boxes_xyxy[i].astype(np.int32)
            crop = frame_bgr[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            color = self._classify_crop(hsv)
            if color is None:
                continue
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
            results.append({
                'u': int(u), 'v': int(v),
                'x': int(x1), 'y': int(y1),
                'w': int(x2 - x1), 'h': int(y2 - y1),
                'type': color,
            })
        return results

    def _classify_crop(self, hsv_crop):
        green_mask = cv2.inRange(hsv_crop,
                                 np.array([35, 60, 60]),
                                 np.array([85, 255, 255]))
        red_mask1 = cv2.inRange(hsv_crop,
                                np.array([0, 120, 70]),
                                np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_crop,
                                np.array([170, 120, 70]),
                                np.array([180, 255, 255]))
        total = hsv_crop.shape[0] * hsv_crop.shape[1]
        if total == 0:
            return None
        green_ratio = np.count_nonzero(green_mask) / total
        red_ratio = (np.count_nonzero(red_mask1) +
                     np.count_nonzero(red_mask2)) / total
        if green_ratio > 0.15:
            return 'waste'
        if red_ratio > 0.15:
            return 'obstacle'
        return None

    def _detect_hsv(self, hsv, lower, upper, color_name):
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 300:
            return None
        x, y, w, h = cv2.boundingRect(largest)
        u = x + w // 2
        v = y + h // 2
        return {'u': int(u), 'v': int(v), 'x': int(x), 'y': int(y),
                'w': int(w), 'h': int(h), 'type': color_name}

    def image_callback(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if self._model is not None:
            detections = self._yolo_infer(frame)
            for det in detections:
                msg_out = String()
                msg_out.data = json.dumps(det)
                if det['type'] == 'waste':
                    self.waste_pub.publish(msg_out)
                    self.get_logger().info(
                        f'[YOLO] Green waste at ({det["u"]}, {det["v"]}) '
                        f'size {det["w"]}x{det["h"]}')
                else:
                    self.bbox_pub.publish(msg_out)
                    self.get_logger().info(
                        f'[YOLO] Red obstacle at ({det["u"]}, {det["v"]}) '
                        f'size {det["w"]}x{det["h"]}')
            if detections:
                return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red = self._detect_hsv(hsv,
                               np.array([0, 120, 70]),
                               np.array([10, 255, 255]), 'obstacle')
        if red is None:
            red = self._detect_hsv(hsv,
                                   np.array([170, 120, 70]),
                                   np.array([180, 255, 255]), 'obstacle')
        green = self._detect_hsv(hsv,
                                 np.array([35, 60, 60]),
                                 np.array([85, 255, 255]), 'waste')
        if red is not None:
            msg_out = String()
            msg_out.data = json.dumps(red)
            self.bbox_pub.publish(msg_out)
            self.get_logger().info(
                f'[HSV] Red obstacle at ({red["u"]}, {red["v"]}) '
                f'size {red["w"]}x{red["h"]}')
        if green is not None:
            msg_out = String()
            msg_out.data = json.dumps(green)
            self.waste_pub.publish(msg_out)
            self.get_logger().info(
                f'[HSV] Green waste at ({green["u"]}, {green["v"]}) '
                f'size {green["w"]}x{green["h"]}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

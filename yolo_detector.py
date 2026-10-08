import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO

import time


class YOLODetector(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.bridge = CvBridge()

        self.model = YOLO(
            "/home/alessia/autonomous-drone-navigation/runs/detect/visdrone_yolov8n/weights/best.pt"
        )

        # Target = veicoli o persone di interesse.
        # VisDrone:
        # 0 pedestrian
        # 3 car
        # 4 van
        # 5 truck
        # 8 bus
        self.target_classes = {
            0: "pedestrian",
            3: "car",
            4: "van",
            5: "truck",
            8: "bus"
        }

        self.image_subscription = self.create_subscription(
            Image,
            "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.image_callback,
            1
        )

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            "/yolo/detections",
            10
        )

        self.min_confidence = 0.10

        self.frame_count = 0
        self.last_log_time = time.time()

        self.get_logger().info(
            "YOLO detector started."
        )
        self.get_logger().info(
            "Target classes: pedestrian, car, van, truck, bus"
        )
        self.get_logger().info(
            f"Minimum confidence: {self.min_confidence:.2f}"
        )

    def image_callback(self, msg):

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            results = self.model(
                frame,
                imgsz=1280,
                conf=self.min_confidence,
                verbose=False
            )

            detection_msg = Detection2DArray()
            detection_msg.header = msg.header

            raw_count = 0
            target_count = 0

            for result in results:

                for box in result.boxes:

                    raw_count += 1

                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    raw_class_name = self.model.names[class_id]

                    if class_id in self.target_classes:

                        target_count += 1
                        class_name = self.target_classes[class_id]

                        self.get_logger().info(
                            f"TARGET: class={class_name} "
                            f"confidence={confidence:.3f}"
                        )

                        x1, y1, x2, y2 = map(
                            float,
                            box.xyxy[0]
                        )

                        detection = Detection2D()
                        detection.header = msg.header

                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = class_name
                        hypothesis.hypothesis.score = confidence

                        detection.results.append(hypothesis)

                        detection.bbox.center.position.x = (
                            (x1 + x2) / 2.0
                        )
                        detection.bbox.center.position.y = (
                            (y1 + y2) / 2.0
                        )

                        detection.bbox.size_x = x2 - x1
                        detection.bbox.size_y = y2 - y1

                        detection_msg.detections.append(
                            detection
                        )

            self.detection_publisher.publish(
                detection_msg
            )

            now = time.time()

            if now - self.last_log_time >= 2.0:
                self.get_logger().info(
                    f"YOLO status: frames={self.frame_count}, "
                    f"raw_boxes={raw_count}, "
                    f"target_boxes={target_count}"
                )
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(
                f"YOLO callback error: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = YOLODetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

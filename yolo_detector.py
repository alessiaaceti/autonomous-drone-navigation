import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2


class YOLODetector(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.bridge = CvBridge()

        self.model = YOLO(
            "/home/alessia/autonomous-drone-navigation/runs/detect/visdrone_yolov8n/weights/best.pt"
        )

        self.target_classes = {
            0: "pedestrian",
            3: "car",
            5: "truck"
        }

        self.image_subscription = self.create_subscription(
            Image,
            "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.image_callback,
            10
        )

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            "/yolo/detections",
            10
        )

        self.get_logger().info(
            "YOLO detector started."
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        results = self.model(
            frame,
            verbose=False
        )

        detection_msg = Detection2DArray()
        detection_msg.header = msg.header

        for result in results:

            for box in result.boxes:

                class_id = int(box.cls[0])

                if class_id not in self.target_classes:
                    continue

                confidence = float(box.conf[0])
                class_name = self.target_classes[class_id]

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

                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0

                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                detection_msg.detections.append(detection)

        self.detection_publisher.publish(
            detection_msg
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
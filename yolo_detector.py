import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2


class YOLODetector(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.bridge = CvBridge()

        self.model = YOLO("yolov8n.pt")

        self.image_subscription = self.create_subscription(
            Image,
            "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.image_callback,
            10
        )

        self.detection_publisher = self.create_publisher(
            String,
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

        detections = []

        for result in results:

            for box in result.boxes:

                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                class_name = self.model.names[class_id]

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0]
                )

                detections.append(
                    f"{class_name}:{confidence:.2f}:"
                    f"{x1},{y1},{x2},{y2}"
                )

        detection_msg = String()

        if detections:
            detection_msg.data = "|".join(detections)
        else:
            detection_msg.data = "NONE"

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
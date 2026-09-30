import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver(Node):

    def __init__(self):
        super().__init__("image_saver")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.image_callback,
            10
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        cv2.imwrite(
            "drone_frame.png",
            frame
        )

        self.get_logger().info(
            "Frame saved to drone_frame.png"
        )

        rclpy.shutdown()


def main():

    rclpy.init()

    node = ImageSaver()

    rclpy.spin(node)

    node.destroy_node()


if __name__ == "__main__":
    main()

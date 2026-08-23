import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class DownwardCameraPublisher(Node):
    def __init__(self):
        super().__init__("downward_camera_publisher")

        self.declare_parameter("video_device", "/dev/video6")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("image_topic", "/downward_camera/image_jpeg")
        self.declare_parameter("frame_id", "downward_camera")

        self.video_device = self.get_parameter("video_device").value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.camera_fps = float(self.get_parameter("camera_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(CompressedImage, self.image_topic, 10)

        self.cap = None
        self.open_camera()

        period = 1.0 / max(self.camera_fps, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f"DOWNWARD_CAMERA_PUBLISHER_READY device={self.video_device}")
        self.get_logger().info(f"IMAGE_TOPIC={self.image_topic}")
        self.get_logger().info(f"IMAGE_SIZE={self.image_width}x{self.image_height}")
        self.get_logger().info(f"FPS={self.camera_fps}")

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()

        device = self.video_device

        if str(device).isdigit():
            device = int(device)

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        if self.cap.isOpened():
            self.get_logger().info(f"CAMERA_OPENED device={self.video_device}")
        else:
            self.get_logger().error(f"CAMERA_OPEN_FAILED device={self.video_device}")

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self.open_camera()
            time.sleep(0.5)
            return

        ok, frame = self.cap.read()

        if not ok or frame is None:
            self.get_logger().warn("CAMERA_FRAME_READ_FAILED")
            return

        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        if not ok:
            self.get_logger().warn("JPEG_ENCODE_FAILED")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main():
    rclpy.init()
    node = DownwardCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

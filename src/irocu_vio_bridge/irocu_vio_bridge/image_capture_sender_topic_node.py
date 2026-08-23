import csv
import math
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class ImageCaptureSenderTopicNode(Node):
    def __init__(self):
        super().__init__("image_capture_sender_topic_node")

        self.declare_parameter("upload_url", "http://10.7.19.98:8000/upload")
        self.declare_parameter("image_topic", "/downward_camera/image_jpeg")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("capture_interval_seconds", 2.0)
        self.declare_parameter("base_dir", str(Path.home() / "isro_workspace"))
        self.declare_parameter("upload_retries", 3)
        self.declare_parameter("upload_timeout", 10.0)

        self.upload_url = self.get_parameter("upload_url").value
        self.image_topic = self.get_parameter("image_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.capture_interval_seconds = float(self.get_parameter("capture_interval_seconds").value)
        self.base_dir = Path(self.get_parameter("base_dir").value)
        self.upload_retries = int(self.get_parameter("upload_retries").value)
        self.upload_timeout = float(self.get_parameter("upload_timeout").value)

        self.capture_dir = self.base_dir / "captured_img"
        self.log_dir = self.base_dir / "logs"
        self.state_dir = self.base_dir / "state"
        self.log_file = self.log_dir / "pi_capture_log.csv"
        self.next_index_file = self.state_dir / "next_index.txt"

        self.latest_pose = None
        self.latest_image_bytes = None
        self.latest_image_stamp = None
        self.is_busy = False

        self.ensure_workspace()

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(CompressedImage, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(self.capture_interval_seconds, self.timer_callback)

        self.get_logger().info(f"IMAGE_CAPTURE_SENDER_TOPIC_READY upload_url={self.upload_url}")
        self.get_logger().info(f"IMAGE_TOPIC={self.image_topic}")
        self.get_logger().info(f"POSE_TOPIC={self.pose_topic}")
        self.get_logger().info(f"CAPTURE_INTERVAL_SECONDS={self.capture_interval_seconds}")

    def ensure_workspace(self):
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.state_dir.mkdir(parents=True, exist_ok=True)

        if not self.log_file.exists():
            with self.log_file.open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["image_name", "x", "y", "z", "yaw", "timestamp", "sent_status"])

    def pose_callback(self, msg):
        self.latest_pose = msg

    def image_callback(self, msg):
        self.latest_image_bytes = bytes(msg.data)
        self.latest_image_stamp = msg.header.stamp

    def timer_callback(self):
        if self.is_busy:
            return

        self.is_busy = True

        try:
            self.capture_and_send_once()
        finally:
            self.is_busy = False

    def load_next_index(self):
        if not self.next_index_file.exists():
            return 0

        try:
            return int(self.next_index_file.read_text().strip())
        except Exception:
            return 0

    def save_next_index(self, index):
        self.next_index_file.write_text(str(index))

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_values_from_msg(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation

        x = round(p.x, 3)
        y = round(p.y, 3)
        z = round(p.z, 3)
        yaw = round(self.quaternion_to_yaw(q), 3)

        return x, y, z, yaw

    def pose_filename(self, index, x, y, z, yaw):
        return f"img_{index:04d}_x{x:.3f}_y{y:.3f}_z{z:.3f}_yaw{yaw:.3f}.jpg"

    def upload_image_once(self, image_path):
        upload_url = f"{self.upload_url}?filename={image_path.name}"

        with image_path.open("rb") as file:
            request = urllib.request.Request(
                upload_url,
                data=file.read(),
                method="POST",
                headers={"Content-Type": "application/octet-stream"},
            )

        with urllib.request.urlopen(request, timeout=self.upload_timeout) as response:
            return response.status, response.read().decode("utf-8", errors="replace")

    def upload_image(self, image_path):
        last_error = None

        for attempt in range(1, self.upload_retries + 1):
            try:
                return self.upload_image_once(image_path)
            except Exception as error:
                last_error = error
                self.get_logger().warn(f"UPLOAD_RETRY {attempt}/{self.upload_retries}: {image_path.name} | {error}")
                time.sleep(0.5)

        raise last_error

    def append_log(self, image_name, x, y, z, yaw, status):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with self.log_file.open("a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([image_name, x, y, z, yaw, timestamp, status])

    def capture_and_send_once(self):
        if self.latest_pose is None:
            self.get_logger().warn("WAITING_FOR_POSE no image sent")
            return False

        if self.latest_image_bytes is None:
            self.get_logger().warn("WAITING_FOR_IMAGE no image sent")
            return False

        index = self.load_next_index()
        x, y, z, yaw = self.pose_values_from_msg(self.latest_pose)

        image_name = self.pose_filename(index, x, y, z, yaw)
        image_path = self.capture_dir / image_name

        image_path.write_bytes(self.latest_image_bytes)

        try:
            status_code, response_text = self.upload_image(image_path)

            if status_code == 200:
                status = "AUTO_SENT"
            else:
                status = f"AUTO_FAILED_{status_code}"

            self.get_logger().info(f"{status}: {image_name} | {response_text}")
        except Exception as error:
            status = "AUTO_UPLOAD_FAILED"
            self.get_logger().error(f"{status}: {image_name} | {error}")

        self.append_log(image_name, x, y, z, yaw, status)
        self.save_next_index(index + 1)

        return True


def main():
    rclpy.init()
    node = ImageCaptureSenderTopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import csv
import math
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class ImageCaptureSenderNode(Node):
    def __init__(self):
        super().__init__("image_capture_sender_node")

        self.declare_parameter("upload_url", "http://10.7.19.98:8000/upload")
        self.declare_parameter("camera_device", "/dev/video1")
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("capture_fps", 30)
        self.declare_parameter("capture_interval_seconds", 2.0)
        self.declare_parameter("base_dir", str(Path.home() / "isro_workspace"))
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("upload_retries", 3)
        self.declare_parameter("upload_timeout", 10.0)

        self.upload_url = self.get_parameter("upload_url").value
        self.camera_device = self.get_parameter("camera_device").value
        self.capture_width = int(self.get_parameter("capture_width").value)
        self.capture_height = int(self.get_parameter("capture_height").value)
        self.capture_fps = int(self.get_parameter("capture_fps").value)
        self.capture_interval_seconds = float(self.get_parameter("capture_interval_seconds").value)
        self.base_dir = Path(self.get_parameter("base_dir").value)
        self.pose_topic = self.get_parameter("pose_topic").value
        self.upload_retries = int(self.get_parameter("upload_retries").value)
        self.upload_timeout = float(self.get_parameter("upload_timeout").value)

        self.capture_dir = self.base_dir / "captured_img"
        self.log_dir = self.base_dir / "logs"
        self.state_dir = self.base_dir / "state"
        self.log_file = self.log_dir / "pi_capture_log.csv"
        self.next_index_file = self.state_dir / "next_index.txt"

        self.latest_pose = None
        self.pose_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.ensure_workspace()

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        self.worker = threading.Thread(target=self.capture_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(f"FOREVER_DOWNWARD_CAMERA_SENDER_READY upload_url={self.upload_url}")
        self.get_logger().info(f"CAMERA_DEVICE={self.camera_device}")
        self.get_logger().info(f"POSE_TOPIC={self.pose_topic}")
        self.get_logger().info(f"CAPTURE_SIZE={self.capture_width}x{self.capture_height}")
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
        with self.pose_lock:
            self.latest_pose = msg

    def get_latest_pose(self):
        with self.pose_lock:
            return self.latest_pose

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
        return f"img_{index:04d}_x{x:.3f}_y{y:.3f}_z{z:.3f}_yaw{yaw:.3f}.png"

    def open_camera(self):
        device = self.camera_device

        if str(device).isdigit():
            device = int(device)

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        cap.set(cv2.CAP_PROP_FPS, self.capture_fps)

        return cap

    def capture_image(self, cap, output_path):
        ok = False
        frame = None

        for _ in range(10):
            ok, frame = cap.read()

        if not ok or frame is None:
            return False, None

        saved = cv2.imwrite(str(output_path), frame)

        if not saved:
            return False, None

        h, w = frame.shape[:2]

        return True, f"{w}x{h}"

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

    def capture_and_send_once(self, cap):
        pose_msg = self.get_latest_pose()

        if pose_msg is None:
            self.get_logger().warn("WAITING_FOR_POSE no image sent")
            return False

        index = self.load_next_index()
        x, y, z, yaw = self.pose_values_from_msg(pose_msg)

        image_name = self.pose_filename(index, x, y, z, yaw)
        image_path = self.capture_dir / image_name

        captured, resolution = self.capture_image(cap, image_path)

        if not captured:
            status = "AUTO_CAPTURE_FAILED"
            self.append_log(image_name, x, y, z, yaw, status)
            self.get_logger().error(f"{status}: {image_name}")
            return False

        try:
            status_code, response_text = self.upload_image(image_path)

            if status_code == 200:
                status = "AUTO_SENT"
            else:
                status = f"AUTO_FAILED_{status_code}"

            self.get_logger().info(f"{status}: {image_name} | {resolution} | {response_text}")
        except Exception as error:
            status = "AUTO_UPLOAD_FAILED"
            self.get_logger().error(f"{status}: {image_name} | {error}")

        self.append_log(image_name, x, y, z, yaw, status)
        self.save_next_index(index + 1)

        return True

    def capture_loop(self):
        cap = None

        try:
            while not self.stop_event.is_set():
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        cap.release()

                    cap = self.open_camera()

                    if not cap.isOpened():
                        self.get_logger().error(f"CAMERA_OPEN_FAILED device={self.camera_device}")
                        time.sleep(1.0)
                        continue

                    self.get_logger().info(f"CAMERA_OPENED device={self.camera_device}")

                self.capture_and_send_once(cap)
                time.sleep(self.capture_interval_seconds)

        finally:
            if cap is not None:
                cap.release()

    def destroy_node(self):
        self.stop_event.set()

        if self.worker.is_alive():
            self.worker.join(timeout=2.0)

        super().destroy_node()


def main():
    rclpy.init()
    node = ImageCaptureSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

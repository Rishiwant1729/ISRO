import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


class Phase3ArucoDetectorCustom(Node):
    def __init__(self):
        super().__init__('phase3_aruco_detector_custom')

        self.declare_parameter('video_device', '/dev/video6')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fps', 15.0)
        self.declare_parameter('aruco_marker_id', 4)
        self.declare_parameter('aruco_marker_size', 0.20)
        self.declare_parameter('landing_camera_offset_x', 0.0)
        self.declare_parameter('landing_camera_offset_y', 0.0)
        self.declare_parameter('camera_rotation_deg', 90.0)
        self.declare_parameter('detect_rate_hz', 10.0)
        self.declare_parameter('min_area', 1200.0)
        self.declare_parameter('max_match_error', 0.28)

        self.video_device = str(self.get_parameter('video_device').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.camera_fps = float(self.get_parameter('camera_fps').value)
        self.aruco_marker_id = int(self.get_parameter('aruco_marker_id').value)
        self.aruco_marker_size = float(self.get_parameter('aruco_marker_size').value)
        self.landing_camera_offset_x = float(self.get_parameter('landing_camera_offset_x').value)
        self.landing_camera_offset_y = float(self.get_parameter('landing_camera_offset_y').value)
        self.camera_rotation_deg = float(self.get_parameter('camera_rotation_deg').value)
        self.detect_rate_hz = float(self.get_parameter('detect_rate_hz').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_match_error = float(self.get_parameter('max_match_error').value)

        self.pub = self.create_publisher(Vector3Stamped, '/landing/aruco_offset', 10)

        self.ref_size = 120
        self.ref_marker = self.make_reference_marker(self.aruco_marker_id, self.ref_size)
        self.ref_markers = [
            self.ref_marker,
            cv2.rotate(self.ref_marker, cv2.ROTATE_90_CLOCKWISE),
            cv2.rotate(self.ref_marker, cv2.ROTATE_180),
            cv2.rotate(self.ref_marker, cv2.ROTATE_90_COUNTERCLOCKWISE)
        ]

        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'CAMERA_OPEN_FAILED device={self.video_device}')
        else:
            self.get_logger().info(f'CAMERA_OPENED device={self.video_device}')

        self.timer = self.create_timer(1.0 / self.detect_rate_hz, self.tick)

        self.get_logger().info(
            f'CUSTOM_ARUCO_DETECTOR_READY device={self.video_device} id={self.aruco_marker_id}'
        )

    def make_reference_marker(self, marker_id, size):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = np.zeros((size, size), dtype=np.uint8)
        cv2.aruco.drawMarker(d, marker_id, size, marker, 1)
        _, marker = cv2.threshold(marker, 127, 255, cv2.THRESH_BINARY)
        return marker

    def order_points(self, pts):
        pts = np.array(pts, dtype=np.float32)
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1).reshape(-1)

        ordered = np.zeros((4, 2), dtype=np.float32)
        ordered[0] = pts[np.argmin(s)]
        ordered[2] = pts[np.argmax(s)]
        ordered[1] = pts[np.argmin(diff)]
        ordered[3] = pts[np.argmax(diff)]

        return ordered

    def match_marker(self, warped):
        gray = cv2.resize(warped, (self.ref_size, self.ref_size))
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        best_error = 1.0

        for ref in self.ref_markers:
            diff = cv2.absdiff(binary, ref)
            error = np.count_nonzero(diff) / float(self.ref_size * self.ref_size)
            best_error = min(best_error, error)

        return best_error

    def detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        thresh = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            7
        )

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_error = 1.0

        for c in contours:
            area = cv2.contourArea(c)

            if area < self.min_area:
                continue

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            if len(approx) != 4:
                continue

            if not cv2.isContourConvex(approx):
                continue

            pts = approx.reshape(4, 2)
            ordered = self.order_points(pts)

            dst = np.array([
                [0, 0],
                [self.ref_size - 1, 0],
                [self.ref_size - 1, self.ref_size - 1],
                [0, self.ref_size - 1]
            ], dtype=np.float32)

            M = cv2.getPerspectiveTransform(ordered, dst)
            warped = cv2.warpPerspective(gray, M, (self.ref_size, self.ref_size))

            error = self.match_marker(warped)

            if error < best_error:
                best_error = error
                best = ordered

        if best is None:
            return None, None

        if best_error > self.max_match_error:
            return None, best_error

        return best, best_error

    def rotate_offsets(self, raw_x, raw_y):
        angle = int(round(self.camera_rotation_deg)) % 360

        if angle == 0:
            return raw_x, raw_y

        if angle == 90:
            return -raw_y, raw_x

        if angle == 180:
            return -raw_x, -raw_y

        if angle == 270:
            return raw_y, -raw_x

        return raw_x, raw_y

    def tick(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn('CAMERA_FRAME_READ_FAILED')
            return

        pts, error = self.detect_marker(frame)

        if pts is None:
            return

        h, w = frame.shape[:2]

        marker_center = np.mean(pts, axis=0)
        image_center_x = w * 0.5
        image_center_y = h * 0.5

        side_lengths = []

        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            side_lengths.append(float(np.linalg.norm(p2 - p1)))

        avg_side_px = max(sum(side_lengths) / len(side_lengths), 1.0)
        meter_per_pixel = self.aruco_marker_size / avg_side_px

        raw_x = (marker_center[0] - image_center_x) * meter_per_pixel
        raw_y = (marker_center[1] - image_center_y) * meter_per_pixel

        rot_x, rot_y = self.rotate_offsets(raw_x, raw_y)

        ox = rot_x - self.landing_camera_offset_x
        oy = rot_y - self.landing_camera_offset_y
        dist = self.aruco_marker_size * 430.0 / avg_side_px

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'landing_camera'
        out.vector.x = float(ox)
        out.vector.y = float(oy)
        out.vector.z = float(dist)

        self.pub.publish(out)

        self.get_logger().info(
            f'CUSTOM_ARUCO_FOUND id={self.aruco_marker_id} ox={ox:.3f} oy={oy:.3f} dist={dist:.2f} error={error:.3f}'
        )

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Phase3ArucoDetectorCustom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

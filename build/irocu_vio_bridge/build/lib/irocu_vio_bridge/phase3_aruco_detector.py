import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped


class Phase3ArucoDetector(Node):
    def __init__(self):
        super().__init__('phase3_aruco_detector')

        self.declare_parameter('image_topic', '/landing_camera/image_raw')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_marker_id', 4)
        self.declare_parameter('aruco_marker_size', 0.20)
        self.declare_parameter('landing_camera_offset_x', 0.0)
        self.declare_parameter('landing_camera_offset_y', 0.0)
        self.declare_parameter('publish_rate_limit_hz', 20.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.aruco_dictionary = str(self.get_parameter('aruco_dictionary').value)
        self.aruco_marker_id = int(self.get_parameter('aruco_marker_id').value)
        self.aruco_marker_size = float(self.get_parameter('aruco_marker_size').value)
        self.landing_camera_offset_x = float(self.get_parameter('landing_camera_offset_x').value)
        self.landing_camera_offset_y = float(self.get_parameter('landing_camera_offset_y').value)
        self.publish_rate_limit_hz = float(self.get_parameter('publish_rate_limit_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub = self.create_publisher(Vector3Stamped, '/landing/aruco_offset', 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionary_id(self.aruco_dictionary))
        self.aruco_params = cv2.aruco.DetectorParameters()

        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.detector = None

        self.last_pub_time = None

        self.get_logger().info(
            f'ARUCO_DETECTOR_READY topic={self.image_topic} dict={self.aruco_dictionary} id={self.aruco_marker_id}'
        )

    def dictionary_id(self, name):
        table = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100
        }
        return table.get(name, cv2.aruco.DICT_4X4_50)

    def limited(self):
        if self.last_pub_time is None:
            return False
        age = (self.get_clock().now() - self.last_pub_time).nanoseconds / 1e9
        return age < (1.0 / self.publish_rate_limit_hz)

    def image_to_frame(self, msg):
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ['bgr8', 'rgb8']:
            frame = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
            if enc == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(frame)

        if enc in ['mono8', '8uc1']:
            gray = data.reshape((msg.height, msg.step))[:, :msg.width]
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            return np.ascontiguousarray(frame)

        if enc in ['yuyv', 'yuyv422', 'yuv422', 'yuv422_yuy2']:
            yuyv = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
            frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
            return np.ascontiguousarray(frame)

        if enc in ['uyvy', 'uyvy422']:
            uyvy = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
            frame = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
            return np.ascontiguousarray(frame)

        self.get_logger().error(f'UNSUPPORTED_ENCODING {msg.encoding}')
        return None

    def image_cb(self, msg):
        if self.limited():
            return

        frame = self.image_to_frame(msg)

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        ids_flat = ids.flatten()

        if self.aruco_marker_id not in ids_flat:
            return

        idx = list(ids_flat).index(self.aruco_marker_id)
        pts = corners[idx][0]

        h, w = gray.shape[:2]
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

        ox = ((marker_center[0] - image_center_x) * meter_per_pixel) - self.landing_camera_offset_x
        oy = ((marker_center[1] - image_center_y) * meter_per_pixel) - self.landing_camera_offset_y
        dist = self.aruco_marker_size * 430.0 / avg_side_px

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'landing_camera'
        out.vector.x = float(ox)
        out.vector.y = float(oy)
        out.vector.z = float(dist)

        self.pub.publish(out)
        self.last_pub_time = self.get_clock().now()

        self.get_logger().info(f'ARUCO_FOUND id={self.aruco_marker_id} ox={ox:.3f} oy={oy:.3f} dist={dist:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = Phase3ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

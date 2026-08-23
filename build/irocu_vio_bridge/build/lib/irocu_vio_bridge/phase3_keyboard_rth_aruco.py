import sys
import time
import math
import select
import termios
import tty
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class Phase3KeyboardRthAruco(Node):
    def __init__(self):
        super().__init__('phase3_keyboard_rth_aruco')

        self.declare_parameter('target_altitude', 0.5)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)

        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('vertical_speed', 0.05)
        self.declare_parameter('yaw_rate', 0.15)
        self.declare_parameter('max_altitude', 1.0)
        self.declare_parameter('min_altitude', 0.30)
        self.declare_parameter('hover_before_control_seconds', 5.0)

        self.declare_parameter('return_speed_max', 0.10)
        self.declare_parameter('return_kp_xy', 0.55)
        self.declare_parameter('return_kp_z', 0.45)
        self.declare_parameter('return_accept_radius', 0.18)
        self.declare_parameter('return_accept_z', 0.15)
        self.declare_parameter('return_timeout_seconds', 35.0)

        self.declare_parameter('yaw_hold_enabled', True)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('max_yaw_rate', 0.25)
        self.declare_parameter('yaw_deadband', 0.03)
        self.declare_parameter('yaw_control_sign', 1.0)

        self.declare_parameter('precision_landing_enabled', True)
        self.declare_parameter('landing_image_topic', '/landing_camera/image_raw')
        self.declare_parameter('landing_image_compressed', False)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_marker_id', 4)
        self.declare_parameter('aruco_marker_size', 0.20)

        self.declare_parameter('aruco_search_timeout_seconds', 20.0)
        self.declare_parameter('aruco_align_timeout_seconds', 35.0)
        self.declare_parameter('aruco_lost_timeout_seconds', 2.0)
        self.declare_parameter('aruco_accept_xy', 0.05)
        self.declare_parameter('aruco_accept_hold_seconds', 2.0)
        self.declare_parameter('aruco_precision_kp_x', 0.35)
        self.declare_parameter('aruco_precision_kp_y', 0.35)
        self.declare_parameter('aruco_precision_speed_max', 0.04)
        self.declare_parameter('aruco_descent_speed', 0.03)
        self.declare_parameter('aruco_final_land_altitude', 0.18)
        self.declare_parameter('aruco_descend_enabled', True)

        self.declare_parameter('aruco_vx_sign', -1.0)
        self.declare_parameter('aruco_vy_sign', -1.0)
        self.declare_parameter('landing_camera_offset_x', 0.0)
        self.declare_parameter('landing_camera_offset_y', 0.0)

        self.declare_parameter('landing_yaw_mode', 'home')
        self.declare_parameter('landing_yaw_offset_deg', 0.0)
        self.declare_parameter('landing_yaw_accept_deg', 5.0)
        self.declare_parameter('landing_yaw_hold_seconds', 2.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.confirm_flight = bool(self.get_parameter('confirm_flight').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_latitude = float(self.get_parameter('takeoff_latitude').value)
        self.takeoff_longitude = float(self.get_parameter('takeoff_longitude').value)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.vertical_speed = float(self.get_parameter('vertical_speed').value)
        self.yaw_rate = float(self.get_parameter('yaw_rate').value)
        self.max_altitude = float(self.get_parameter('max_altitude').value)
        self.min_altitude = float(self.get_parameter('min_altitude').value)
        self.hover_before_control_seconds = float(self.get_parameter('hover_before_control_seconds').value)

        self.return_speed_max = float(self.get_parameter('return_speed_max').value)
        self.return_kp_xy = float(self.get_parameter('return_kp_xy').value)
        self.return_kp_z = float(self.get_parameter('return_kp_z').value)
        self.return_accept_radius = float(self.get_parameter('return_accept_radius').value)
        self.return_accept_z = float(self.get_parameter('return_accept_z').value)
        self.return_timeout_seconds = float(self.get_parameter('return_timeout_seconds').value)

        self.yaw_hold_enabled = bool(self.get_parameter('yaw_hold_enabled').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.yaw_control_sign = float(self.get_parameter('yaw_control_sign').value)

        self.precision_landing_enabled = bool(self.get_parameter('precision_landing_enabled').value)
        self.landing_image_topic = str(self.get_parameter('landing_image_topic').value)
        self.landing_image_compressed = bool(self.get_parameter('landing_image_compressed').value)
        self.aruco_dictionary = str(self.get_parameter('aruco_dictionary').value)
        self.aruco_marker_id = int(self.get_parameter('aruco_marker_id').value)
        self.aruco_marker_size = float(self.get_parameter('aruco_marker_size').value)

        self.aruco_search_timeout_seconds = float(self.get_parameter('aruco_search_timeout_seconds').value)
        self.aruco_align_timeout_seconds = float(self.get_parameter('aruco_align_timeout_seconds').value)
        self.aruco_lost_timeout_seconds = float(self.get_parameter('aruco_lost_timeout_seconds').value)
        self.aruco_accept_xy = float(self.get_parameter('aruco_accept_xy').value)
        self.aruco_accept_hold_seconds = float(self.get_parameter('aruco_accept_hold_seconds').value)
        self.aruco_precision_kp_x = float(self.get_parameter('aruco_precision_kp_x').value)
        self.aruco_precision_kp_y = float(self.get_parameter('aruco_precision_kp_y').value)
        self.aruco_precision_speed_max = float(self.get_parameter('aruco_precision_speed_max').value)
        self.aruco_descent_speed = float(self.get_parameter('aruco_descent_speed').value)
        self.aruco_final_land_altitude = float(self.get_parameter('aruco_final_land_altitude').value)
        self.aruco_descend_enabled = bool(self.get_parameter('aruco_descend_enabled').value)

        self.aruco_vx_sign = float(self.get_parameter('aruco_vx_sign').value)
        self.aruco_vy_sign = float(self.get_parameter('aruco_vy_sign').value)
        self.landing_camera_offset_x = float(self.get_parameter('landing_camera_offset_x').value)
        self.landing_camera_offset_y = float(self.get_parameter('landing_camera_offset_y').value)

        self.landing_yaw_mode = str(self.get_parameter('landing_yaw_mode').value)
        self.landing_yaw_offset_deg = float(self.get_parameter('landing_yaw_offset_deg').value)
        self.landing_yaw_accept_deg = float(self.get_parameter('landing_yaw_accept_deg').value)
        self.landing_yaw_hold_seconds = float(self.get_parameter('landing_yaw_hold_seconds').value)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vio_time = None
        self.local_time = None
        self.state_time = None
        self.state = None
        self.local_pose = None
        self.target_pose = None

        self.start_z = None
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_yaw = None
        self.landing_yaw = None
        self.mission_done = False

        self.aruco_active = False
        self.aruco_visible = False
        self.aruco_last_seen_time = None
        self.aruco_offset_x = 0.0
        self.aruco_offset_y = 0.0
        self.aruco_distance = 0.0
        self.aruco_image_width = 0
        self.aruco_image_height = 0

        self.create_subscription(Odometry, '/vio/odometry', self.vio_cb, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb, best_effort_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, reliable_qos)

        if self.landing_image_compressed:
            self.create_subscription(CompressedImage, self.landing_image_topic, self.compressed_image_cb, best_effort_qos)
        else:
            self.create_subscription(Image, self.landing_image_topic, self.image_cb, best_effort_qos)

        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', reliable_qos)
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', reliable_qos)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionary_id(self.aruco_dictionary))
        self.aruco_params = cv2.aruco.DetectorParameters()

        try:
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_detector = None

        self.get_logger().info(
            f'ARUCO_CONFIG dictionary={self.aruco_dictionary} center_id={self.aruco_marker_id} marker_size={self.aruco_marker_size:.3f}'
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

    def vio_cb(self, msg):
        self.vio_time = self.get_clock().now()

    def local_cb(self, msg):
        self.local_pose = msg
        self.local_time = self.get_clock().now()

    def state_cb(self, msg):
        self.state = msg
        self.state_time = self.get_clock().now()

    def image_cb(self, msg):
        if not self.aruco_active:
            return

        try:
            enc = msg.encoding.lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if enc in ['bgr8', 'rgb8']:
                frame = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
                if enc == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            elif enc in ['mono8', '8uc1']:
                gray = data.reshape((msg.height, msg.step))[:, :msg.width]
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            elif enc in ['yuyv', 'yuyv422', 'yuv422', 'yuv422_yuy2']:
                yuyv = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
                frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

            elif enc in ['uyvy', 'uyvy422']:
                uyvy = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
                frame = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)

            else:
                self.get_logger().error(f'ARUCO_UNSUPPORTED_IMAGE_ENCODING {msg.encoding}')
                return

            frame = np.ascontiguousarray(frame)
            self.process_aruco_frame(frame)

        except Exception as e:
            self.get_logger().error(f'ARUCO_IMAGE_ERROR {e}')

    def compressed_image_cb(self, msg):
        if not self.aruco_active:
            return
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.process_aruco_frame(frame)
        except Exception as e:
            self.get_logger().error(f'ARUCO_COMPRESSED_IMAGE_ERROR {e}')

    def process_aruco_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        self.aruco_visible = False

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

        raw_offset_x = (marker_center[0] - image_center_x) * meter_per_pixel
        raw_offset_y = (marker_center[1] - image_center_y) * meter_per_pixel

        self.aruco_offset_x = raw_offset_x - self.landing_camera_offset_x
        self.aruco_offset_y = raw_offset_y - self.landing_camera_offset_y
        self.aruco_distance = self.aruco_marker_size * 430.0 / avg_side_px
        self.aruco_image_width = w
        self.aruco_image_height = h
        self.aruco_visible = True
        self.aruco_last_seen_time = self.get_clock().now()

    def fresh(self, t, max_age=2.0):
        if t is None:
            return False
        age = (self.get_clock().now() - t).nanoseconds / 1e9
        return age <= max_age

    def spin_for(self, seconds):
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True
        return False

    def wait_for_service(self, client, name):
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {name}')
        return rclpy.ok()

    def call_service(self, client, req, name):
        if not self.wait_for_service(client, name):
            return None
        future = client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()

    def health_ok(self, require_disarmed=True):
        vio_ok = self.fresh(self.vio_time)
        local_ok = self.fresh(self.local_time)
        state_ok = self.fresh(self.state_time)

        if self.state is None:
            self.get_logger().info(
                f'vio={vio_ok} local_position={local_ok} state=False connected=False guided=False disarmed=False mode=None'
            )
            return False

        connected_ok = self.state.connected
        guided_ok = self.state.guided and self.state.mode == 'GUIDED'
        disarmed_ok = not self.state.armed

        self.get_logger().info(
            f'vio={vio_ok} local_position={local_ok} state={state_ok} '
            f'connected={connected_ok} guided={guided_ok} disarmed={disarmed_ok} '
            f'armed={self.state.armed} mode={self.state.mode}'
        )

        if require_disarmed:
            return vio_ok and local_ok and state_ok and connected_ok and guided_ok and disarmed_ok

        return vio_ok and local_ok and state_ok and connected_ok and guided_ok

    def wait_for_health(self, timeout_sec=15.0, require_disarmed=True):
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.health_ok(require_disarmed=require_disarmed):
                self.get_logger().info('HEALTH_PASS')
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error('HEALTH_TIMEOUT')
                return False

    def set_mode(self, mode):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        res = self.call_service(self.mode_client, req, '/mavros/set_mode')
        if res is None:
            return False
        self.get_logger().info(f'SET_MODE {mode} mode_sent={res.mode_sent}')
        self.spin_for(1.0)
        return res.mode_sent

    def arm(self, value):
        req = CommandBool.Request()
        req.value = value
        res = self.call_service(self.arming_client, req, '/mavros/cmd/arming')
        if res is None:
            return False
        self.get_logger().info(f'ARM value={value} success={res.success} result={res.result}')
        self.spin_for(1.0)
        return res.success

    def takeoff_command(self):
        for attempt in range(1, 4):
            self.set_mode('GUIDED')
            self.spin_for(1.0)

            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = self.takeoff_latitude
            req.longitude = self.takeoff_longitude
            req.altitude = self.target_altitude

            res = self.call_service(self.takeoff_client, req, '/mavros/cmd/takeoff')

            if res is None:
                self.get_logger().error(f'TAKEOFF attempt={attempt} no_response')
                self.spin_for(2.0)
                continue

            self.get_logger().info(f'TAKEOFF attempt={attempt} success={res.success} result={res.result}')

            if res.success:
                return True

            self.spin_for(2.0)

        return False

    def make_target(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = self.local_pose.pose.orientation
        return msg

    def publish_position_for(self, seconds):
        if self.target_pose is None:
            return False

        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.position_pub.publish(self.target_pose)

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True

            time.sleep(period)

        return False

    def climb_and_hold(self, start_z):
        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y
        target_z = start_z + self.target_altitude
        self.target_pose = self.make_target(x, y, target_z)

        self.get_logger().info(f'PUBLISHING_TAKEOFF_SETPOINT z={target_z:.2f}')
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.position_pub.publish(self.target_pose)

            if self.local_pose is not None:
                dz = self.local_pose.pose.position.z - start_z
                self.get_logger().info(f'altitude_delta={dz:.2f} target={self.target_altitude:.2f}')
                if dz >= self.target_altitude * 0.92:
                    return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > 18.0:
                self.get_logger().error('TAKEOFF_SETPOINT_TIMEOUT')
                return False

            time.sleep(1.0 / self.setpoint_rate_hz)

        return False

    def publish_zero_velocity_for(self, seconds):
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.velocity_pub.publish(Twist())

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True

            time.sleep(period)

        return False

    def safe_land_disarm(self):
        self.get_logger().info('SAFE_LAND_DISARM')
        self.publish_zero_velocity_for(1.0)
        self.set_mode('LAND')
        self.spin_for(8.0)
        self.arm(False)
        self.mission_done = True

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_wrap(self, value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def current_yaw(self):
        if self.local_pose is None:
            return None
        return self.quaternion_to_yaw(self.local_pose.pose.orientation)

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def set_landing_yaw_from_home(self):
        if self.home_yaw is None:
            self.landing_yaw = self.current_yaw()
        else:
            self.landing_yaw = self.angle_wrap(self.home_yaw + math.radians(self.landing_yaw_offset_deg))
        self.get_logger().info(f'LANDING_YAW_SET yaw={self.landing_yaw:.3f} offset_deg={self.landing_yaw_offset_deg:.1f}')

    def yaw_error(self):
        yaw = self.current_yaw()
        if yaw is None or self.landing_yaw is None:
            return None
        return self.angle_wrap(self.landing_yaw - yaw)

    def yaw_is_safe(self):
        err = self.yaw_error()
        if err is None:
            return False
        return abs(math.degrees(err)) <= self.landing_yaw_accept_deg

    def yaw_hold_rate(self):
        if not self.yaw_hold_enabled:
            return 0.0

        if self.landing_yaw is None:
            return 0.0

        err = self.yaw_error()

        if err is None:
            return 0.0

        if abs(err) < self.yaw_deadband:
            return 0.0

        return self.clamp(self.yaw_control_sign * self.yaw_kp * err, self.max_yaw_rate)

    def get_key(self):
        if not sys.stdin.isatty():
            return ''

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)

        if readable:
            return sys.stdin.read(1)

        return ''

    def velocity_from_key(self, key):
        msg = Twist()
        key = key.lower()

        if key == 'w':
            msg.linear.y = -self.linear_speed
        elif key == 's':
            msg.linear.y = self.linear_speed
        elif key == 'a':
            msg.linear.x = self.linear_speed
        elif key == 'd':
            msg.linear.x = -self.linear_speed
        elif key == 'r':
            if self.local_pose is not None and self.start_z is not None:
                altitude = self.local_pose.pose.position.z - self.start_z
                if altitude < self.max_altitude:
                    msg.linear.z = self.vertical_speed
        elif key == 'f':
            if self.local_pose is not None and self.start_z is not None:
                altitude = self.local_pose.pose.position.z - self.start_z
                if altitude > self.min_altitude:
                    msg.linear.z = -self.vertical_speed
        elif key == 'z':
            msg.angular.z = self.yaw_rate
        elif key == 'x':
            msg.angular.z = -self.yaw_rate
        else:
            return Twist()

        return msg

    def print_keyboard_help(self):
        print('')
        print('========== PHASE 3 KEYBOARD RTH ARUCO ==========')
        print('W = forward')
        print('S = backward')
        print('A = left')
        print('D = right')
        print('R = up')
        print('F = down')
        print('Z = yaw left')
        print('X = yaw right')
        print('SPACE = stop')
        print('H = return home, align yaw, find center marker ID 4, land')
        print('L = land')
        print('Q = land and exit')
        print('================================================')
        print('Keep this terminal focused while flying.')
        print('')

    def return_home(self):
        if self.home_x is None or self.home_y is None or self.home_z is None:
            self.get_logger().error('RTH_ABORT_NO_HOME')
            return False

        if self.local_pose is None:
            self.get_logger().error('RTH_ABORT_NO_LOCAL_POSE')
            return False

        target_z = self.start_z + self.target_altitude
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()

        self.get_logger().info(
            f'RTH_START home_x={self.home_x:.2f} home_y={self.home_y:.2f} target_z={target_z:.2f}'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error('RTH_ABORT_STATE_NOT_VALID')
                self.publish_zero_velocity_for(1.0)
                return False

            if self.local_pose is None:
                self.get_logger().error('RTH_ABORT_NO_LOCAL_POSE')
                self.publish_zero_velocity_for(1.0)
                return False

            x = self.local_pose.pose.position.x
            y = self.local_pose.pose.position.y
            z = self.local_pose.pose.position.z

            ex = self.home_x - x
            ey = self.home_y - y
            ez = target_z - z

            dist_xy = math.sqrt(ex * ex + ey * ey)
            dist_z = abs(ez)
            yaw_err = self.yaw_error()
            yaw_err_deg = math.degrees(yaw_err) if yaw_err is not None else 999.0

            self.get_logger().info(
                f'RTH_PROGRESS dist_xy={dist_xy:.2f} dist_z={dist_z:.2f} yaw_err_deg={yaw_err_deg:.1f} ex={ex:.2f} ey={ey:.2f} ez={ez:.2f}'
            )

            if dist_xy <= self.return_accept_radius and dist_z <= self.return_accept_z and self.yaw_is_safe():
                self.get_logger().info('RTH_REACHED_HOME_AND_SAFE_YAW')
                self.publish_zero_velocity_for(2.0)
                return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if elapsed >= self.return_timeout_seconds:
                self.get_logger().error('RTH_TIMEOUT')
                self.publish_zero_velocity_for(1.0)
                return False

            msg = Twist()
            msg.linear.x = self.clamp(self.return_kp_xy * ex, self.return_speed_max)
            msg.linear.y = self.clamp(self.return_kp_xy * ey, self.return_speed_max)
            msg.linear.z = self.clamp(self.return_kp_z * ez, self.return_speed_max)
            msg.angular.z = self.yaw_hold_rate()

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def aruco_seen_recently(self):
        if self.aruco_last_seen_time is None:
            return False
        age = (self.get_clock().now() - self.aruco_last_seen_time).nanoseconds / 1e9
        return age <= self.aruco_lost_timeout_seconds

    def aruco_search(self):
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()

        self.get_logger().info('ARUCO_SEARCH_START')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.aruco_visible and self.aruco_seen_recently():
                self.get_logger().info(
                    f'ARUCO_FOUND id={self.aruco_marker_id} ox={self.aruco_offset_x:.3f} oy={self.aruco_offset_y:.3f} distance={self.aruco_distance:.2f}'
                )
                self.publish_zero_velocity_for(1.0)
                return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if elapsed >= self.aruco_search_timeout_seconds:
                self.get_logger().error('ARUCO_SEARCH_TIMEOUT')
                self.publish_zero_velocity_for(1.0)
                return False

            msg = Twist()
            msg.angular.z = self.yaw_hold_rate()
            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def aruco_align(self):
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()
        centered_since = None

        self.get_logger().info('ARUCO_ALIGN_START')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if not self.aruco_visible or not self.aruco_seen_recently():
                self.get_logger().warn('ARUCO_ALIGN_MARKER_LOST')
                self.publish_zero_velocity_for(1.0)
                return False

            ox = self.aruco_offset_x
            oy = self.aruco_offset_y
            xy_centered = abs(ox) <= self.aruco_accept_xy and abs(oy) <= self.aruco_accept_xy
            yaw_safe = self.yaw_is_safe()
            yaw_err = self.yaw_error()
            yaw_err_deg = math.degrees(yaw_err) if yaw_err is not None else 999.0
            ready = xy_centered and yaw_safe

            if ready:
                if centered_since is None:
                    centered_since = self.get_clock().now()
                hold_time = (self.get_clock().now() - centered_since).nanoseconds / 1e9
                if hold_time >= max(self.aruco_accept_hold_seconds, self.landing_yaw_hold_seconds):
                    self.get_logger().info(
                        f'ARUCO_CENTERED_AND_YAW_SAFE ox={ox:.3f} oy={oy:.3f} yaw_err_deg={yaw_err_deg:.1f} hold={hold_time:.1f}'
                    )
                    self.publish_zero_velocity_for(1.0)
                    return True
            else:
                centered_since = None

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if elapsed >= self.aruco_align_timeout_seconds:
                self.get_logger().error('ARUCO_ALIGN_TIMEOUT')
                self.publish_zero_velocity_for(1.0)
                return False

            msg = Twist()
            msg.linear.x = self.clamp(self.aruco_vx_sign * self.aruco_precision_kp_x * ox, self.aruco_precision_speed_max)
            msg.linear.y = self.clamp(self.aruco_vy_sign * self.aruco_precision_kp_y * oy, self.aruco_precision_speed_max)
            msg.linear.z = 0.0
            msg.angular.z = self.yaw_hold_rate()

            self.get_logger().info(
                f'ARUCO_ALIGN ox={ox:.3f} oy={oy:.3f} xy_ok={xy_centered} yaw_err_deg={yaw_err_deg:.1f} yaw_ok={yaw_safe} vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} wz={msg.angular.z:.3f}'
            )

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def aruco_descend(self):
        period = 1.0 / self.setpoint_rate_hz
        lost_start = None

        self.get_logger().info('ARUCO_DESCENT_START')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error('ARUCO_DESCENT_ABORT_STATE_NOT_VALID')
                self.publish_zero_velocity_for(1.0)
                return False

            if self.local_pose is None or self.start_z is None:
                self.get_logger().error('ARUCO_DESCENT_ABORT_NO_LOCAL_POSE')
                self.publish_zero_velocity_for(1.0)
                return False

            altitude = self.local_pose.pose.position.z - self.start_z

            if altitude <= self.aruco_final_land_altitude:
                self.get_logger().info(f'ARUCO_FINAL_LAND altitude={altitude:.2f}')
                self.publish_zero_velocity_for(1.0)
                return True

            marker_ok = self.aruco_visible and self.aruco_seen_recently()
            yaw_safe = self.yaw_is_safe()
            yaw_err = self.yaw_error()
            yaw_err_deg = math.degrees(yaw_err) if yaw_err is not None else 999.0

            msg = Twist()

            if marker_ok:
                lost_start = None
                ox = self.aruco_offset_x
                oy = self.aruco_offset_y
                xy_safe = abs(ox) <= self.aruco_accept_xy * 1.8 and abs(oy) <= self.aruco_accept_xy * 1.8

                msg.linear.x = self.clamp(self.aruco_vx_sign * self.aruco_precision_kp_x * ox, self.aruco_precision_speed_max)
                msg.linear.y = self.clamp(self.aruco_vy_sign * self.aruco_precision_kp_y * oy, self.aruco_precision_speed_max)

                if xy_safe and yaw_safe:
                    msg.linear.z = -self.aruco_descent_speed
                else:
                    msg.linear.z = 0.0

                self.get_logger().info(
                    f'ARUCO_DESCEND alt={altitude:.2f} ox={ox:.3f} oy={oy:.3f} xy_safe={xy_safe} yaw_err_deg={yaw_err_deg:.1f} yaw_safe={yaw_safe} vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} vz={msg.linear.z:.3f}'
                )
            else:
                if lost_start is None:
                    lost_start = self.get_clock().now()

                lost_time = (self.get_clock().now() - lost_start).nanoseconds / 1e9
                msg.linear.z = 0.0
                self.get_logger().warn(f'ARUCO_DESCEND_MARKER_LOST lost_time={lost_time:.1f}')

                if lost_time >= self.aruco_lost_timeout_seconds:
                    self.get_logger().error('ARUCO_DESCENT_MARKER_LOST_TIMEOUT')
                    self.publish_zero_velocity_for(1.0)
                    return False

            msg.angular.z = self.yaw_hold_rate()
            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def precision_land_on_aruco(self):
        self.get_logger().info('ARUCO_PRECISION_LANDING_START')

        self.aruco_active = True
        self.aruco_visible = False
        self.aruco_last_seen_time = None

        self.publish_zero_velocity_for(2.0)

        if not self.aruco_search():
            self.get_logger().error('ARUCO_PRECISION_FALLBACK_LAND_AFTER_SEARCH_FAIL')
            self.safe_land_disarm()
            return False

        if not self.aruco_align():
            self.get_logger().error('ARUCO_PRECISION_FALLBACK_LAND_AFTER_ALIGN_FAIL')
            self.safe_land_disarm()
            return False

        if not self.aruco_descend_enabled:
            self.get_logger().warn('ARUCO_DESCEND_DISABLED_SAFE_LAND_FROM_ALIGNED_POSITION')
            self.safe_land_disarm()
            return True

        if not self.aruco_descend():
            self.get_logger().error('ARUCO_PRECISION_FALLBACK_LAND_AFTER_DESCEND_FAIL')
            self.safe_land_disarm()
            return False

        self.get_logger().info('ARUCO_SWITCHING_TO_LAND')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_KEYBOARD_RTH_ARUCO_DONE')
        return True

    def keyboard_control_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error('Keyboard terminal is not interactive')
            return False

        old_settings = termios.tcgetattr(sys.stdin)
        period = 1.0 / self.setpoint_rate_hz

        self.print_keyboard_help()

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok() and not self.mission_done:
                rclpy.spin_once(self, timeout_sec=0.0)

                if self.state is None:
                    self.get_logger().error('NO_STATE_DURING_KEYBOARD_CONTROL')
                    return False

                if not self.state.armed or self.state.mode != 'GUIDED':
                    self.get_logger().error(
                        f'KEYBOARD_ABORTED_STATE_NOT_VALID armed={self.state.armed} '
                        f'guided={self.state.guided} mode={self.state.mode}'
                    )
                    return False

                key = self.get_key()

                if key == 'q' or key == 'Q':
                    self.get_logger().info('KEY_Q_LAND_EXIT')
                    self.safe_land_disarm()
                    return True

                if key == 'l' or key == 'L':
                    self.get_logger().info('KEY_L_LAND')
                    self.safe_land_disarm()
                    return True

                if key == 'h' or key == 'H':
                    self.get_logger().info('KEY_H_RTH_ARUCO_LAND')
                    self.set_landing_yaw_from_home()
                    self.publish_zero_velocity_for(1.0)

                    if not self.return_home():
                        self.safe_land_disarm()
                        return False

                    if self.precision_landing_enabled:
                        self.precision_land_on_aruco()
                    else:
                        self.safe_land_disarm()

                    return True

                if key == ' ':
                    msg = Twist()
                else:
                    msg = self.velocity_from_key(key)

                self.velocity_pub.publish(msg)

                if self.local_pose is not None and self.start_z is not None:
                    altitude = self.local_pose.pose.position.z - self.start_z
                    self.get_logger().info(
                        f'key={repr(key)} vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} '
                        f'vz={msg.linear.z:.2f} wz={msg.angular.z:.2f} alt={altitude:.2f}'
                    )

                time.sleep(period)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.publish_zero_velocity_for(1.0)

        return True

    def run(self):
        if not self.confirm_flight:
            self.get_logger().error('confirm_flight is false. Not starting motors.')
            return

        if not self.wait_for_health(require_disarmed=True):
            return

        if not self.set_mode('GUIDED'):
            return

        if not self.wait_for_health(require_disarmed=True):
            return

        start_x = self.local_pose.pose.position.x
        start_y = self.local_pose.pose.position.y
        start_z = self.local_pose.pose.position.z

        self.start_z = start_z
        self.home_x = start_x
        self.home_y = start_y
        self.home_z = start_z
        self.home_yaw = self.current_yaw()
        self.set_landing_yaw_from_home()

        self.get_logger().info(
            f'HOME_SAVED x={self.home_x:.2f} y={self.home_y:.2f} z={self.home_z:.2f} home_yaw={self.home_yaw:.3f}'
        )

        self.target_pose = self.make_target(start_x, start_y, start_z)

        self.get_logger().info('PRESTREAMING_POSITION_SETPOINTS')
        self.publish_position_for(2.0)

        if not self.arm(True):
            return

        self.spin_for(2.0)

        if self.state is None or not self.state.armed:
            self.get_logger().error(
                f'ARMED_STATE_NOT_CONFIRMED armed={self.state.armed if self.state else None} '
                f'mode={self.state.mode if self.state else None}'
            )
            self.safe_land_disarm()
            return

        if not self.set_mode('GUIDED'):
            self.safe_land_disarm()
            return

        self.spin_for(1.0)

        if not self.takeoff_command():
            self.safe_land_disarm()
            return

        self.spin_for(2.0)

        if not self.climb_and_hold(start_z):
            self.safe_land_disarm()
            return

        hold_x = self.local_pose.pose.position.x
        hold_y = self.local_pose.pose.position.y
        hold_z = start_z + self.target_altitude
        self.target_pose = self.make_target(hold_x, hold_y, hold_z)

        self.get_logger().info(f'HOVER_BEFORE_KEYBOARD_CONTROL {self.hover_before_control_seconds} seconds')
        self.publish_position_for(self.hover_before_control_seconds)

        self.keyboard_control_loop()


def main(args=None):
    rclpy.init(args=args)
    node = Phase3KeyboardRthAruco()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().error('CTRL_C_RECEIVED_SAFE_LAND')
        node.safe_land_disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

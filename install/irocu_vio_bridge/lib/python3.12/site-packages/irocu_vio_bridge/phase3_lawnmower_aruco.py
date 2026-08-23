import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class Phase3LawnmowerAruco(Node):
    def __init__(self):
        super().__init__('phase3_lawnmower_aruco')

        self.declare_parameter('target_altitude', 0.8)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)

        self.declare_parameter('max_length', 1.5)
        self.declare_parameter('max_breadth', 0.8)
        self.declare_parameter('lane_spacing', 0.35)
        self.declare_parameter('boundary_margin', 0.20)
        self.declare_parameter('geofence_tolerance', 0.20)
        self.declare_parameter('geofence_recovery_tolerance', 0.60)
        self.declare_parameter('min_segment_distance', 0.08)
        self.declare_parameter('max_segment_seconds', 60.0)
        self.declare_parameter('waypoint_kp_xy', 0.55)
        self.declare_parameter('waypoint_accept_radius', 0.15)
        self.declare_parameter('waypoint_speed_max', 0.10)

        self.declare_parameter('hover_before_pattern_seconds', 8.0)
        self.declare_parameter('hold_between_segments_seconds', 3.0)
        self.declare_parameter('hover_after_pattern_seconds', 8.0)

        self.declare_parameter('return_home_enabled', True)
        self.declare_parameter('return_speed_max', 0.08)
        self.declare_parameter('return_kp_xy', 0.50)
        self.declare_parameter('return_kp_z', 0.40)
        self.declare_parameter('return_accept_radius', 0.18)
        self.declare_parameter('return_accept_z', 0.15)
        self.declare_parameter('return_timeout_seconds', 60.0)

        self.declare_parameter('yaw_hold_enabled', True)
        self.declare_parameter('yaw_kp', 0.45)
        self.declare_parameter('max_yaw_rate', 0.15)
        self.declare_parameter('yaw_deadband', 0.04)
        self.declare_parameter('yaw_control_sign', 1.0)

        self.declare_parameter('precision_landing_enabled', True)
        self.declare_parameter('aruco_marker_id', 4)
        self.declare_parameter('aruco_search_timeout_seconds', 35.0)
        self.declare_parameter('aruco_align_timeout_seconds', 90.0)
        self.declare_parameter('aruco_lost_timeout_seconds', 1.0)
        self.declare_parameter('aruco_accept_xy', 0.10)
        self.declare_parameter('aruco_accept_hold_seconds', 2.0)
        self.declare_parameter('aruco_precision_kp_x', 0.02)
        self.declare_parameter('aruco_precision_kp_y', 0.02)
        self.declare_parameter('aruco_precision_speed_max', 0.001)
        self.declare_parameter('aruco_descent_speed', 0.006)
        self.declare_parameter('aruco_final_land_altitude', 0.35)
        self.declare_parameter('aruco_vx_sign', -1.0)
        self.declare_parameter('aruco_vy_sign', -1.0)

        self.declare_parameter('landing_yaw_offset_deg', 0.0)
        self.declare_parameter('landing_yaw_accept_deg', 6.0)
        self.declare_parameter('landing_yaw_hold_seconds', 2.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.confirm_flight = bool(self.get_parameter('confirm_flight').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_latitude = float(self.get_parameter('takeoff_latitude').value)
        self.takeoff_longitude = float(self.get_parameter('takeoff_longitude').value)

        self.max_length = float(self.get_parameter('max_length').value)
        self.max_breadth = float(self.get_parameter('max_breadth').value)
        self.lane_spacing = float(self.get_parameter('lane_spacing').value)
        self.boundary_margin = float(self.get_parameter('boundary_margin').value)
        self.geofence_tolerance = float(self.get_parameter('geofence_tolerance').value)
        self.geofence_recovery_tolerance = float(self.get_parameter('geofence_recovery_tolerance').value)
        self.min_segment_distance = float(self.get_parameter('min_segment_distance').value)
        self.max_segment_seconds = float(self.get_parameter('max_segment_seconds').value)
        self.waypoint_kp_xy = float(self.get_parameter('waypoint_kp_xy').value)
        self.waypoint_accept_radius = float(self.get_parameter('waypoint_accept_radius').value)
        self.waypoint_speed_max = float(self.get_parameter('waypoint_speed_max').value)

        self.hover_before_pattern_seconds = float(self.get_parameter('hover_before_pattern_seconds').value)
        self.hold_between_segments_seconds = float(self.get_parameter('hold_between_segments_seconds').value)
        self.hover_after_pattern_seconds = float(self.get_parameter('hover_after_pattern_seconds').value)

        self.return_home_enabled = bool(self.get_parameter('return_home_enabled').value)
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
        self.aruco_marker_id = int(self.get_parameter('aruco_marker_id').value)
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
        self.aruco_vx_sign = float(self.get_parameter('aruco_vx_sign').value)
        self.aruco_vy_sign = float(self.get_parameter('aruco_vy_sign').value)

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

        self.create_subscription(Odometry, '/vio/odometry', self.vio_cb, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb, best_effort_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, reliable_qos)
        self.create_subscription(Vector3Stamped, '/landing/aruco_offset', self.aruco_offset_cb, reliable_qos)

        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', reliable_qos)
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', reliable_qos)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    def vio_cb(self, msg):
        self.vio_time = self.get_clock().now()

    def local_cb(self, msg):
        self.local_pose = msg
        self.local_time = self.get_clock().now()

    def state_cb(self, msg):
        self.state = msg
        self.state_time = self.get_clock().now()

    def aruco_offset_cb(self, msg):
        if not self.aruco_active:
            return
        self.aruco_offset_x = float(msg.vector.x)
        self.aruco_offset_y = float(msg.vector.y)
        self.aruco_distance = float(msg.vector.z)
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
            self.get_logger().info(f'vio={vio_ok} local_position={local_ok} state=False connected=False guided=False disarmed=False mode=None')
            return False

        connected_ok = self.state.connected
        guided_ok = self.state.guided and self.state.mode == 'GUIDED'
        disarmed_ok = not self.state.armed

        self.get_logger().info(
            f'vio={vio_ok} local_position={local_ok} state={state_ok} connected={connected_ok} guided={guided_ok} disarmed={disarmed_ok} armed={self.state.armed} mode={self.state.mode}'
        )

        if require_disarmed:
            return vio_ok and local_ok and state_ok and connected_ok and guided_ok and disarmed_ok

        return vio_ok and local_ok and state_ok and connected_ok and guided_ok

    def wait_for_health(self, timeout_sec=20.0, require_disarmed=True):
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
        """Smooth velocity-based climb to target altitude"""
        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y
        target_z = start_z + self.target_altitude
        
        climb_speed = 0.4  # m/s
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()
        held_at_target = 0.0
        
        # Position hold gains to prevent drift
        kp_xy = 0.3
        max_xy_speed = 0.2

        self.get_logger().info(f'SMOOTH_VELOCITY_CLIMB_START target_z={target_z:.2f}')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.local_pose is None:
                time.sleep(period)
                continue

            current_z = self.local_pose.pose.position.z
            remaining = target_z - current_z

            msg = Twist()

            # Hold X and Y exactly at the takeoff spot to prevent drift
            ex = x - self.local_pose.pose.position.x
            ey = y - self.local_pose.pose.position.y
            msg.linear.x = self.clamp(kp_xy * ex, max_xy_speed)
            msg.linear.y = self.clamp(kp_xy * ey, max_xy_speed)

            # Altitude control
            if remaining > 0.05:
                if remaining < 0.3:
                    # Slow down smoothly when within 30cm of target
                    msg.linear.z = remaining * 1.2  
                else:
                    # Climb at steady speed
                    msg.linear.z = climb_speed
                held_at_target = 0.0
            else:
                # Reached target altitude
                msg.linear.z = 0.0
                held_at_target += period
                
                # Hover for 2 seconds to prove it's stable
                if held_at_target >= 2.0:
                    self.get_logger().info('SMOOTH_CLIMB_COMPLETE')
                    self.publish_zero_velocity_for(1.0)
                    return True

            msg.angular.z = self.yaw_hold_rate()
            self.velocity_pub.publish(msg)

            self.get_logger().info(
                f'CLIMB z={current_z:.2f} remaining={remaining:.2f} vz={msg.linear.z:.3f}'
            )

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > 30.0:
                self.get_logger().error('CLIMB_TIMEOUT')
                return False

            time.sleep(period)
            
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

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

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

    def scan_limits(self):
        scan_length = max(self.max_length - self.boundary_margin, self.min_segment_distance)
        scan_breadth = max(self.max_breadth - self.boundary_margin, self.min_segment_distance)

        min_x = self.home_x - scan_breadth
        max_x = self.home_x
        min_y = self.home_y - scan_length
        max_y = self.home_y

        return min_x, max_x, min_y, max_y

    def inside_scan_boundary(self, extra=0.0):
        if self.local_pose is None or self.home_x is None or self.home_y is None:
            return False

        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y

        min_x, max_x, min_y, max_y = self.scan_limits()

        return (
            x >= min_x - extra and
            x <= max_x + extra and
            y >= min_y - extra and
            y <= max_y + extra
        )

    def clamp_target_to_boundary(self, x, y):
        min_x, max_x, min_y, max_y = self.scan_limits()
        return max(min_x, min(max_x, x)), max(min_y, min(max_y, y))

    def plan_lawnmower_waypoints(self):
        min_x, max_x, min_y, max_y = self.scan_limits()

        x_values = []
        x = max_x

        while x > min_x:
            x_values.append(x)
            x -= self.lane_spacing

        if len(x_values) == 0 or abs(x_values[-1] - min_x) >= self.min_segment_distance:
            x_values.append(min_x)

        waypoints = []

        for i, x in enumerate(x_values):
            y_target = min_y if i % 2 == 0 else max_y
            waypoints.append(self.clamp_target_to_boundary(x, y_target))

            if i < len(x_values) - 1:
                waypoints.append(self.clamp_target_to_boundary(x_values[i + 1], y_target))

        clean = []

        for wp in waypoints:
            if len(clean) == 0:
                clean.append(wp)
                continue

            dx = wp[0] - clean[-1][0]
            dy = wp[1] - clean[-1][1]
            if math.sqrt(dx * dx + dy * dy) >= self.min_segment_distance:
                clean.append(wp)

        self.get_logger().info(
            f'LAWNMOWER_WAYPOINT_PLAN_CREATED count={len(clean)} min_x={min_x:.2f} max_x={max_x:.2f} min_y={min_y:.2f} max_y={max_y:.2f}'
        )

        for i, wp in enumerate(clean):
            self.get_logger().info(f'WAYPOINT_{i + 1} x={wp[0]:.2f} y={wp[1]:.2f}')

        return clean

    def move_to_xy(self, target_x, target_y, label):
        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()
        target_z = self.start_z + self.target_altitude

        self.get_logger().info(f'MOVE_TO_XY_START label={label} target_x={target_x:.2f} target_y={target_y:.2f}')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error(f'MOVE_TO_XY_ABORT_STATE_NOT_VALID label={label}')
                self.publish_zero_velocity_for(1.0)
                return False

            if self.local_pose is None:
                self.get_logger().error(f'MOVE_TO_XY_ABORT_NO_LOCAL_POSE label={label}')
                self.publish_zero_velocity_for(1.0)
                return False

            if not self.inside_scan_boundary(extra=self.geofence_recovery_tolerance):
                x = self.local_pose.pose.position.x
                y = self.local_pose.pose.position.y
                min_x, max_x, min_y, max_y = self.scan_limits()
                self.get_logger().error(
                    f'GEOFENCE_ABORT x={x:.2f} y={y:.2f} allowed_x=[{min_x - self.geofence_recovery_tolerance:.2f},{max_x + self.geofence_recovery_tolerance:.2f}] allowed_y=[{min_y - self.geofence_recovery_tolerance:.2f},{max_y + self.geofence_recovery_tolerance:.2f}]'
                )
                self.publish_zero_velocity_for(1.0)
                return False

            x = self.local_pose.pose.position.x
            y = self.local_pose.pose.position.y
            z = self.local_pose.pose.position.z

            ex = target_x - x
            ey = target_y - y
            ez = target_z - z
            dist_xy = math.sqrt(ex * ex + ey * ey)

            self.get_logger().info(
                f'MOVE_TO_XY_PROGRESS label={label} dist_xy={dist_xy:.2f} ex={ex:.2f} ey={ey:.2f} ez={ez:.2f}'
            )

            if dist_xy <= self.waypoint_accept_radius:
                self.get_logger().info(f'MOVE_TO_XY_DONE label={label}')
                self.publish_zero_velocity_for(self.hold_between_segments_seconds)
                return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if elapsed >= self.max_segment_seconds:
                self.get_logger().error(f'MOVE_TO_XY_TIMEOUT label={label}')
                self.publish_zero_velocity_for(1.0)
                return False

            msg = Twist()
            msg.linear.x = self.clamp(self.waypoint_kp_xy * ex, self.waypoint_speed_max)
            msg.linear.y = self.clamp(self.waypoint_kp_xy * ey, self.waypoint_speed_max)
            msg.linear.z = self.clamp(self.return_kp_z * ez, self.waypoint_speed_max)
            msg.angular.z = self.yaw_hold_rate()

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

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

        self.get_logger().info(f'RTH_START home_x={self.home_x:.2f} home_y={self.home_y:.2f} target_z={target_z:.2f}')

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

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if elapsed >= self.aruco_align_timeout_seconds:
                self.get_logger().error('ARUCO_ALIGN_TIMEOUT')
                self.publish_zero_velocity_for(1.0)
                return False

            if xy_centered:
                if yaw_safe:
                    if centered_since is None:
                        centered_since = self.get_clock().now()

                    hold_time = (self.get_clock().now() - centered_since).nanoseconds / 1e9

                    self.get_logger().info(
                        f'ARUCO_HOLD_CENTERED ox={ox:.3f} oy={oy:.3f} yaw_err_deg={yaw_err_deg:.1f} hold={hold_time:.1f}'
                    )

                    if hold_time >= max(self.aruco_accept_hold_seconds, self.landing_yaw_hold_seconds):
                        self.get_logger().info(
                            f'ARUCO_CENTERED_AND_YAW_SAFE ox={ox:.3f} oy={oy:.3f} yaw_err_deg={yaw_err_deg:.1f} hold={hold_time:.1f}'
                        )
                        self.publish_zero_velocity_for(1.0)
                        return True
                else:
                    centered_since = None

                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = self.yaw_hold_rate()
                self.velocity_pub.publish(msg)
                time.sleep(period)
                continue

            centered_since = None

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

                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = self.yaw_hold_rate()

                self.get_logger().warn(f'ARUCO_DESCEND_MARKER_LOST lost_time={lost_time:.1f}')

                if lost_time >= self.aruco_lost_timeout_seconds:
                    self.get_logger().error('ARUCO_DESCENT_MARKER_LOST_TIMEOUT')
                    self.publish_zero_velocity_for(1.0)
                    return False

                self.velocity_pub.publish(msg)
                time.sleep(period)
                continue

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

        if not self.aruco_descend():
            self.get_logger().error('ARUCO_PRECISION_FALLBACK_LAND_AFTER_DESCEND_FAIL')
            self.safe_land_disarm()
            return False

        self.get_logger().info('ARUCO_SWITCHING_TO_LAND')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_LAWNMOWER_ARUCO_DONE')
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
                f'ARMED_STATE_NOT_CONFIRMED armed={self.state.armed if self.state else None} mode={self.state.mode if self.state else None}'
            )
            self.safe_land_disarm()
            return

        if not self.set_mode('GUIDED'):
            self.safe_land_disarm()
            return

        self.spin_for(1.0)

        # --- THE MICRO-TAKEOFF TRICK ---
        # ArduPilot ignores velocity commands on the ground. We MUST send a CommandTOL
        # to break the ground lock. We lie and say go to 0.1m so the jerk is minimal.
        self.get_logger().info('SENDING_MICRO_TAKEOFF (0.1m) TO BREAK GROUND LOCK')
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = self.takeoff_latitude
        req.longitude = self.takeoff_longitude
        req.altitude = 0.4  # Only 10cm! Prevents the violent jerk.

        res = self.call_service(self.takeoff_client, req, '/mavros/cmd/takeoff')
        if res is None or not res.success:
            self.get_logger().error('MICRO_TAKEOFF_FAILED')
            self.safe_land_disarm()
            return

        # Wait until drone crosses 5cm so we know it's officially airborne
        self.get_logger().info('WAITING TO CROSS 5CM...')
        micro_start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.local_pose is not None and (self.local_pose.pose.position.z - start_z) > 0.05:
                break
            if (self.get_clock().now() - micro_start).nanoseconds / 1e9 > 5.0:
                self.get_logger().error('MICRO_TAKEOFF_STUCK')
                self.safe_land_disarm()
                return
                
        self.get_logger().info('AIRBORNE! SWITCHING TO SMOOTH VELOCITY CONTROL')
        self.spin_for(0.5) # Let it settle for half a second
        # --------------------------------

        if not self.climb_and_hold(start_z):
            self.safe_land_disarm()
            return

        hold_x = self.local_pose.pose.position.x
        hold_y = self.local_pose.pose.position.y
        hold_z = start_z + self.target_altitude
        self.target_pose = self.make_target(hold_x, hold_y, hold_z)

        self.get_logger().info(f'HOVER_BEFORE_PATTERN {self.hover_before_pattern_seconds} seconds')
        self.publish_position_for(self.hover_before_pattern_seconds)

        waypoints = self.plan_lawnmower_waypoints()

        for i, wp in enumerate(waypoints):
            if not self.move_to_xy(wp[0], wp[1], f'WP_{i + 1}'):
                self.safe_land_disarm()
                return

        self.get_logger().info('LAWNMOWER_DONE_FULL_AREA_COVERED')
        self.publish_zero_velocity_for(self.hover_after_pattern_seconds)

        if self.return_home_enabled:
            self.set_landing_yaw_from_home()
            if not self.return_home():
                self.safe_land_disarm()
                return

        if self.precision_landing_enabled:
            self.precision_land_on_aruco()
        else:
            self.safe_land_disarm()


def main(args=None):
    rclpy.init(args=args)
    node = Phase3LawnmowerAruco()

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

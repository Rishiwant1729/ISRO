import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phase3SquareYawHold(Node):
    def __init__(self):
        super().__init__('phase3_square_yaw_hold')

        self.declare_parameter('target_altitude', 0.5)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)
        self.declare_parameter('hover_before_square_seconds', 5.0)
        self.declare_parameter('hover_after_square_seconds', 5.0)
        self.declare_parameter('hold_between_segments_seconds', 3.0)
        self.declare_parameter('move_speed', 0.08)
        self.declare_parameter('side_length', 0.4)
        self.declare_parameter('max_segment_seconds', 8.0)
        self.declare_parameter('yaw_hold_enabled', True)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('max_yaw_rate', 0.25)
        self.declare_parameter('yaw_deadband', 0.03)
        self.declare_parameter('yaw_control_sign', 1.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.confirm_flight = bool(self.get_parameter('confirm_flight').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_latitude = float(self.get_parameter('takeoff_latitude').value)
        self.takeoff_longitude = float(self.get_parameter('takeoff_longitude').value)
        self.hover_before_square_seconds = float(self.get_parameter('hover_before_square_seconds').value)
        self.hover_after_square_seconds = float(self.get_parameter('hover_after_square_seconds').value)
        self.hold_between_segments_seconds = float(self.get_parameter('hold_between_segments_seconds').value)
        self.move_speed = float(self.get_parameter('move_speed').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.max_segment_seconds = float(self.get_parameter('max_segment_seconds').value)
        self.yaw_hold_enabled = bool(self.get_parameter('yaw_hold_enabled').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.yaw_control_sign = float(self.get_parameter('yaw_control_sign').value)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vio_time = None
        self.local_time = None
        self.state_time = None
        self.state = None
        self.local_pose = None
        self.target_pose = None
        self.target_yaw = None
        self.target_orientation = None

        self.create_subscription(Odometry, '/vio/odometry', self.vio_cb, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb, best_effort_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, reliable_qos)

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

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def lock_yaw(self):
        if self.local_pose is None:
            return False

        self.target_orientation = self.local_pose.pose.orientation
        self.target_yaw = self.quat_to_yaw(self.target_orientation)

        self.get_logger().info(
            f'YAW_LOCKED target_yaw_rad={self.target_yaw:.3f} '
            f'target_yaw_deg={math.degrees(self.target_yaw):.1f}'
        )

        return True

    def yaw_correction(self):
        if not self.yaw_hold_enabled:
            return 0.0

        if self.local_pose is None or self.target_yaw is None:
            return 0.0

        current_yaw = self.quat_to_yaw(self.local_pose.pose.orientation)
        error = self.normalize_angle(self.target_yaw - current_yaw)

        if abs(error) < self.yaw_deadband:
            return 0.0

        correction = self.yaw_control_sign * self.yaw_kp * error
        return self.clamp(correction, -self.max_yaw_rate, self.max_yaw_rate)

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

        if self.target_orientation is not None:
            msg.pose.orientation = self.target_orientation
        else:
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

        self.get_logger().info(f'PUBLISHING TAKEOFF SETPOINT z={target_z:.2f}')
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

            msg = Twist()
            msg.angular.z = self.yaw_correction()
            self.velocity_pub.publish(msg)

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True

            time.sleep(period)

        return False

    def hold_current_position(self, seconds):
        if self.local_pose is None:
            return False

        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y
        z = self.local_pose.pose.position.z
        self.target_pose = self.make_target(x, y, z)

        self.get_logger().info(f'HOLD_POSITION x={x:.2f} y={y:.2f} z={z:.2f} seconds={seconds:.1f}')
        return self.publish_position_for(seconds)

    def velocity_for_direction(self, direction):
        msg = Twist()

        if direction == 'forward':
            msg.linear.y = -self.move_speed
        elif direction == 'right':
            msg.linear.x = -self.move_speed
        elif direction == 'backward':
            msg.linear.y = self.move_speed
        elif direction == 'left':
            msg.linear.x = self.move_speed
        else:
            return None

        msg.linear.z = 0.0
        msg.angular.z = self.yaw_correction()
        return msg

    def axis_distance_for_direction(self, direction, start_x, start_y):
        if self.local_pose is None:
            return 0.0

        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y

        if direction in ['left', 'right']:
            return abs(x - start_x)

        return abs(y - start_y)

    def move_segment(self, direction):
        if self.local_pose is None:
            return False

        start_x = self.local_pose.pose.position.x
        start_y = self.local_pose.pose.position.y
        start_z = self.local_pose.pose.position.z

        period = 1.0 / self.setpoint_rate_hz
        start_time = self.get_clock().now()

        self.get_logger().info(
            f'START_SEGMENT direction={direction} side_length={self.side_length:.2f} speed={self.move_speed:.2f}'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None:
                self.get_logger().error('SEGMENT_ABORTED_NO_STATE')
                self.publish_zero_velocity_for(2.0)
                return False

            if not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error(
                    f'SEGMENT_ABORTED_STATE_NOT_VALID armed={self.state.armed} '
                    f'guided={self.state.guided} mode={self.state.mode}'
                )
                self.publish_zero_velocity_for(2.0)
                return False

            if self.local_pose is None:
                self.get_logger().error('SEGMENT_ABORTED_NO_LOCAL_POSE')
                self.publish_zero_velocity_for(2.0)
                return False

            x = self.local_pose.pose.position.x
            y = self.local_pose.pose.position.y
            z = self.local_pose.pose.position.z

            dx = x - start_x
            dy = y - start_y
            dz = z - start_z
            axis_moved = self.axis_distance_for_direction(direction, start_x, start_y)
            horizontal_moved = math.sqrt(dx * dx + dy * dy)

            current_yaw = self.quat_to_yaw(self.local_pose.pose.orientation)
            yaw_error = 0.0

            if self.target_yaw is not None:
                yaw_error = self.normalize_angle(self.target_yaw - current_yaw)

            yaw_rate = self.yaw_correction()

            self.get_logger().info(
                f'direction={direction} axis_moved={axis_moved:.2f} horizontal={horizontal_moved:.2f} '
                f'dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} '
                f'yaw_error_deg={math.degrees(yaw_error):.1f} yaw_rate={yaw_rate:.2f}'
            )

            if axis_moved >= self.side_length:
                self.get_logger().info(f'SEGMENT_DONE direction={direction}')
                self.publish_zero_velocity_for(2.0)
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= self.max_segment_seconds:
                self.get_logger().error(f'SEGMENT_TIMEOUT direction={direction}')
                self.publish_zero_velocity_for(2.0)
                return False

            msg = self.velocity_for_direction(direction)

            if msg is None:
                self.get_logger().error(f'INVALID_DIRECTION direction={direction}')
                self.publish_zero_velocity_for(2.0)
                return False

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def safe_land_disarm(self):
        self.get_logger().info('SAFE_LAND_DISARM')
        self.publish_zero_velocity_for(1.0)
        self.set_mode('LAND')
        self.spin_for(8.0)
        self.arm(False)

    def run_square(self):
        directions = ['forward', 'right', 'backward', 'left']

        for direction in directions:
            if not self.move_segment(direction):
                return False

            if not self.hold_current_position(self.hold_between_segments_seconds):
                return False

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

        self.target_pose = self.make_target(start_x, start_y, start_z)

        self.get_logger().info('PRESTREAMING POSITION SETPOINTS')
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

        if not self.lock_yaw():
            self.safe_land_disarm()
            return

        hold_x = self.local_pose.pose.position.x
        hold_y = self.local_pose.pose.position.y
        hold_z = start_z + self.target_altitude
        self.target_pose = self.make_target(hold_x, hold_y, hold_z)

        self.get_logger().info(f'HOVER_BEFORE_SQUARE {self.hover_before_square_seconds} seconds')
        self.publish_position_for(self.hover_before_square_seconds)

        if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
            self.get_logger().error(
                f'ABORT_BEFORE_SQUARE armed={self.state.armed if self.state else None} '
                f'guided={self.state.guided if self.state else None} '
                f'mode={self.state.mode if self.state else None}'
            )
            self.safe_land_disarm()
            return

        if not self.run_square():
            self.safe_land_disarm()
            return

        self.get_logger().info(f'HOVER_AFTER_SQUARE {self.hover_after_square_seconds} seconds')
        self.hold_current_position(self.hover_after_square_seconds)

        self.get_logger().info('SWITCHING_TO_LAND')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_SQUARE_YAW_HOLD_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Phase3SquareYawHold()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import sys
import time
import math
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phase3KeyboardRTH(Node):
    def __init__(self):
        super().__init__('phase3_keyboard_rth')

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
        self.declare_parameter('land_after_rth', False)
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
        self.land_after_rth = bool(self.get_parameter('land_after_rth').value)
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
        self.start_z = None
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_yaw = None
        self.running_keyboard = False

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

    def save_home(self):
        if self.local_pose is None:
            return False

        self.home_x = self.local_pose.pose.position.x
        self.home_y = self.local_pose.pose.position.y
        self.home_z = self.local_pose.pose.position.z
        self.home_yaw = self.quat_to_yaw(self.local_pose.pose.orientation)

        self.get_logger().info(
            f'HOME_SAVED x={self.home_x:.2f} y={self.home_y:.2f} z={self.home_z:.2f} '
            f'yaw_deg={math.degrees(self.home_yaw):.1f}'
        )

        return True

    def yaw_correction_to_home(self):
        if not self.yaw_hold_enabled:
            return 0.0

        if self.local_pose is None or self.home_yaw is None:
            return 0.0

        current_yaw = self.quat_to_yaw(self.local_pose.pose.orientation)
        error = self.normalize_angle(self.home_yaw - current_yaw)

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
                current_altitude = self.local_pose.pose.position.z - self.start_z
                if current_altitude < self.max_altitude:
                    msg.linear.z = self.vertical_speed
        elif key == 'f':
            if self.local_pose is not None and self.start_z is not None:
                current_altitude = self.local_pose.pose.position.z - self.start_z
                if current_altitude > self.min_altitude:
                    msg.linear.z = -self.vertical_speed
        elif key == 'z':
            msg.angular.z = self.yaw_rate
        elif key == 'x':
            msg.angular.z = -self.yaw_rate
        else:
            return Twist()

        return msg

    def return_home(self):
        if self.home_x is None or self.home_y is None or self.home_z is None:
            self.get_logger().error('RTH_ABORTED_HOME_NOT_SAVED')
            return False

        period = 1.0 / self.setpoint_rate_hz
        start_time = self.get_clock().now()

        self.get_logger().info(
            f'RTH_STARTED home_x={self.home_x:.2f} home_y={self.home_y:.2f} home_z={self.home_z:.2f}'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error(
                    f'RTH_ABORTED_STATE_NOT_VALID armed={self.state.armed if self.state else None} '
                    f'guided={self.state.guided if self.state else None} '
                    f'mode={self.state.mode if self.state else None}'
                )
                self.publish_zero_velocity_for(2.0)
                return False

            if self.local_pose is None:
                self.get_logger().error('RTH_ABORTED_NO_LOCAL_POSE')
                self.publish_zero_velocity_for(2.0)
                return False

            x = self.local_pose.pose.position.x
            y = self.local_pose.pose.position.y
            z = self.local_pose.pose.position.z

            error_x = self.home_x - x
            error_y = self.home_y - y
            error_z = self.home_z - z

            distance_xy = math.sqrt(error_x * error_x + error_y * error_y)
            yaw_rate_cmd = self.yaw_correction_to_home()

            self.get_logger().info(
                f'RTH distance_xy={distance_xy:.2f} error_x={error_x:.2f} '
                f'error_y={error_y:.2f} error_z={error_z:.2f} yaw_rate={yaw_rate_cmd:.2f}'
            )

            if distance_xy <= self.return_accept_radius and abs(error_z) <= self.return_accept_z:
                self.get_logger().info('RTH_REACHED_HOME')
                self.publish_zero_velocity_for(1.5)
                self.target_pose = self.make_target(self.home_x, self.home_y, self.home_z)
                self.publish_position_for(3.0)
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= self.return_timeout_seconds:
                self.get_logger().error('RTH_TIMEOUT')
                self.publish_zero_velocity_for(2.0)
                return False

            msg = Twist()

            if abs(error_x) > self.return_accept_radius * 0.4:
                msg.linear.x = self.clamp(self.return_kp_xy * error_x, -self.return_speed_max, self.return_speed_max)

            if abs(error_y) > self.return_accept_radius * 0.4:
                msg.linear.y = self.clamp(self.return_kp_xy * error_y, -self.return_speed_max, self.return_speed_max)

            if abs(error_z) > self.return_accept_z * 0.5:
                msg.linear.z = self.clamp(self.return_kp_z * error_z, -self.vertical_speed, self.vertical_speed)

            msg.angular.z = yaw_rate_cmd

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def print_keyboard_help(self):
        print('')
        print('========== PHASE 3 KEYBOARD + RTH ==========')
        print('W = forward')
        print('S = backward')
        print('A = left')
        print('D = right')
        print('R = up')
        print('F = down')
        print('Z = yaw left')
        print('X = yaw right')
        print('SPACE = stop')
        print('H = return to home')
        print('L = land')
        print('Q = land and exit')
        print('============================================')
        print('Keep this terminal focused while flying.')
        print('')

    def keyboard_control_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error('Keyboard terminal is not interactive')
            return False

        old_settings = termios.tcgetattr(sys.stdin)
        period = 1.0 / self.setpoint_rate_hz

        self.running_keyboard = True
        self.print_keyboard_help()

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok() and self.running_keyboard:
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
                    self.running_keyboard = False
                    return True

                if key == 'l' or key == 'L':
                    self.get_logger().info('KEY_L_LAND')
                    self.running_keyboard = False
                    return True

                if key == 'h' or key == 'H':
                    self.get_logger().info('KEY_H_RTH')
                    self.publish_zero_velocity_for(1.0)

                    if not self.return_home():
                        return False

                    if self.land_after_rth:
                        self.get_logger().info('LAND_AFTER_RTH_ENABLED')
                        self.running_keyboard = False
                        return True

                    self.get_logger().info('RTH_DONE_BACK_TO_KEYBOARD')
                    continue

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

        if not self.save_home():
            self.safe_land_disarm()
            return

        hold_x = self.local_pose.pose.position.x
        hold_y = self.local_pose.pose.position.y
        hold_z = start_z + self.target_altitude
        self.target_pose = self.make_target(hold_x, hold_y, hold_z)

        self.get_logger().info(f'HOVER_BEFORE_KEYBOARD_CONTROL {self.hover_before_control_seconds} seconds')
        self.publish_position_for(self.hover_before_control_seconds)

        self.keyboard_control_loop()

        self.get_logger().info('LANDING_AFTER_KEYBOARD_CONTROL')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_KEYBOARD_RTH_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Phase3KeyboardRTH()

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

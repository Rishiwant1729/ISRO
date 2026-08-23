import sys
import time
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


class Phase3KeyboardControl(Node):
    def __init__(self):
        super().__init__('phase3_keyboard_control')

        self.declare_parameter('target_altitude', 0.8)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('vertical_speed', 0.08)
        self.declare_parameter('yaw_rate', 0.25)
        self.declare_parameter('max_altitude', 1.2)
        self.declare_parameter('min_altitude', 0.35)
        self.declare_parameter('hover_before_control_seconds', 5.0)

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

    def health_ok(self, require_disarmed=True):
        vio_ok = self.fresh(self.vio_time)
        local_ok = self.fresh(self.local_time)
        state_ok = self.fresh(self.state_time)

        if self.state is None:
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
            msg.linear.x = max(-max_xy_speed, min(max_xy_speed, kp_xy * ex))
            msg.linear.y = max(-max_xy_speed, min(max_xy_speed, kp_xy * ey))

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

    def print_keyboard_help(self):
        print('')
        print('========== PHASE 3 KEYBOARD CONTROL ==========')
        print('W = forward')
        print('S = backward')
        print('A = left')
        print('D = right')
        print('R = up')
        print('F = down')
        print('Z = yaw left')
        print('X = yaw right')
        print('SPACE = stop')
        print('L = land')
        print('Q = land and exit')
        print('==============================================')
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
        req.altitude = 0.25  # Only 10cm! Prevents the violent jerk.

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

        self.get_logger().info(f'HOVER_BEFORE_KEYBOARD_CONTROL {self.hover_before_control_seconds} seconds')
        self.publish_position_for(self.hover_before_control_seconds)

        self.keyboard_control_loop()

        self.get_logger().info('LANDING_AFTER_KEYBOARD_CONTROL')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_KEYBOARD_CONTROL_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Phase3KeyboardControl()

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

import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phase3VelocityTest(Node):
    def __init__(self):
        super().__init__('phase3_velocity_test')

        self.declare_parameter('target_altitude', 0.5)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)
        self.declare_parameter('hover_before_move_seconds', 3.0)
        self.declare_parameter('hover_after_move_seconds', 3.0)
        self.declare_parameter('move_speed', 0.15)
        self.declare_parameter('move_distance', 0.4)
        self.declare_parameter('max_move_seconds', 6.0)
        self.declare_parameter('move_axis', 'x')
        self.declare_parameter('move_direction', 1.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.confirm_flight = bool(self.get_parameter('confirm_flight').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_latitude = float(self.get_parameter('takeoff_latitude').value)
        self.takeoff_longitude = float(self.get_parameter('takeoff_longitude').value)
        self.hover_before_move_seconds = float(self.get_parameter('hover_before_move_seconds').value)
        self.hover_after_move_seconds = float(self.get_parameter('hover_after_move_seconds').value)
        self.move_speed = float(self.get_parameter('move_speed').value)
        self.move_distance = float(self.get_parameter('move_distance').value)
        self.max_move_seconds = float(self.get_parameter('max_move_seconds').value)
        self.move_axis = str(self.get_parameter('move_axis').value)
        self.move_direction = float(self.get_parameter('move_direction').value)

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

    def takeoff_command(self):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = self.takeoff_latitude
        req.longitude = self.takeoff_longitude
        req.altitude = self.target_altitude

        res = self.call_service(self.takeoff_client, req, '/mavros/cmd/takeoff')
        if res is None:
            return False

        self.get_logger().info(f'TAKEOFF success={res.success} result={res.result}')
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
                if dz >= self.target_altitude * 0.75:
                    return True

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > 15.0:
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
            self.velocity_pub.publish(msg)

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True

            time.sleep(period)

        return False

    def move_with_velocity(self):
        if self.local_pose is None:
            return False

        start_x = self.local_pose.pose.position.x
        start_y = self.local_pose.pose.position.y
        start_z = self.local_pose.pose.position.z

        period = 1.0 / self.setpoint_rate_hz
        start_time = self.get_clock().now()

        self.get_logger().info(
            f'STARTING_VELOCITY_MOVE axis={self.move_axis} direction={self.move_direction} '
            f'speed={self.move_speed:.2f} distance={self.move_distance:.2f}'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.state is None or not self.state.armed or self.state.mode != 'GUIDED':
                self.get_logger().error('MOVE_ABORTED_STATE_NOT_VALID')
                self.publish_zero_velocity_for(2.0)
                return False

            if self.local_pose is None:
                self.get_logger().error('MOVE_ABORTED_NO_LOCAL_POSE')
                self.publish_zero_velocity_for(2.0)
                return False

            x = self.local_pose.pose.position.x
            y = self.local_pose.pose.position.y
            z = self.local_pose.pose.position.z

            dx = x - start_x
            dy = y - start_y
            dz = z - start_z
            horizontal_distance = math.sqrt(dx * dx + dy * dy)

            self.get_logger().info(
                f'moved={horizontal_distance:.2f} dx={dx:.2f} dy={dy:.2f} dz={dz:.2f}'
            )

            if horizontal_distance >= self.move_distance:
                self.get_logger().info('MOVE_DISTANCE_REACHED')
                self.publish_zero_velocity_for(2.0)
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= self.max_move_seconds:
                self.get_logger().error('MOVE_TIMEOUT')
                self.publish_zero_velocity_for(2.0)
                return False

            msg = Twist()

            if self.move_axis == 'x':
                msg.linear.x = self.move_speed * self.move_direction
            elif self.move_axis == 'y':
                msg.linear.y = self.move_speed * self.move_direction
            else:
                self.get_logger().error('INVALID_MOVE_AXIS')
                self.publish_zero_velocity_for(2.0)
                return False

            msg.linear.z = 0.0
            msg.angular.z = 0.0

            self.velocity_pub.publish(msg)
            time.sleep(period)

        return False

    def safe_land_disarm(self):
        self.publish_zero_velocity_for(1.0)
        self.set_mode('LAND')
        self.spin_for(8.0)
        self.arm(False)

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

        self.spin_for(1.0)

        if not self.takeoff_command():
            self.safe_land_disarm()
            return

        self.spin_for(2.0)

        if not self.climb_and_hold(start_z):
            self.safe_land_disarm()
            return

        self.get_logger().info(f'HOVER_BEFORE_MOVE {self.hover_before_move_seconds} seconds')
        self.publish_position_for(self.hover_before_move_seconds)

        if not self.move_with_velocity():
            self.safe_land_disarm()
            return

        hold_x = self.local_pose.pose.position.x
        hold_y = self.local_pose.pose.position.y
        hold_z = start_z + self.target_altitude
        self.target_pose = self.make_target(hold_x, hold_y, hold_z)

        self.get_logger().info(f'HOVER_AFTER_MOVE {self.hover_after_move_seconds} seconds')
        self.publish_position_for(self.hover_after_move_seconds)

        self.get_logger().info('SWITCHING_TO_LAND')
        self.safe_land_disarm()
        self.get_logger().info('PHASE3_VELOCITY_TEST_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Phase3VelocityTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

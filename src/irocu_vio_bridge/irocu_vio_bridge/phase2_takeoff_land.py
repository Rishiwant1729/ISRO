import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phase2TakeoffLand(Node):
    def __init__(self):
        super().__init__('phase2_takeoff_land')

        self.declare_parameter('target_altitude', 0.5)
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('confirm_flight', False)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_latitude', 28.0)
        self.declare_parameter('takeoff_longitude', 77.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        self.confirm_flight = bool(self.get_parameter('confirm_flight').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_latitude = float(self.get_parameter('takeoff_latitude').value)
        self.takeoff_longitude = float(self.get_parameter('takeoff_longitude').value)

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

        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', reliable_qos)

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
            f'mode={self.state.mode}'
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

    def make_target(self, z):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.local_pose.pose.position.x
        msg.pose.position.y = self.local_pose.pose.position.y
        msg.pose.position.z = z
        msg.pose.orientation = self.local_pose.pose.orientation
        return msg

    def publish_target_for(self, seconds):
        if self.target_pose is None:
            return False

        period = 1.0 / self.setpoint_rate_hz
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return True

            time.sleep(period)

        return False

    def climb_and_hold(self, start_z):
        target_z = start_z + self.target_altitude
        self.target_pose = self.make_target(target_z)

        self.get_logger().info(f'PUBLISHING TAKEOFF SETPOINT z={target_z:.2f}')
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)

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

        start_z = self.local_pose.pose.position.z
        self.target_pose = self.make_target(start_z)

        self.get_logger().info('PRESTREAMING SETPOINTS')
        self.publish_target_for(2.0)

        if not self.arm(True):
            return

        self.spin_for(1.0)

        if not self.takeoff_command():
            self.set_mode('LAND')
            self.spin_for(5.0)
            self.arm(False)
            return

        self.spin_for(2.0)

        if not self.climb_and_hold(start_z):
            self.set_mode('LAND')
            self.spin_for(5.0)
            self.arm(False)
            return

        self.get_logger().info(f'HOVERING {self.hover_seconds} seconds')
        self.publish_target_for(self.hover_seconds)

        self.get_logger().info('SWITCHING TO LAND')
        self.set_mode('LAND')
        self.spin_for(8.0)
        self.arm(False)
        self.get_logger().info('MISSION_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Phase2TakeoffLand()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

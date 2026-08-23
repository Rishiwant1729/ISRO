import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phase2HealthCheck(Node):
    def __init__(self):
        super().__init__('phase2_health_check')

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

        self.create_subscription(Odometry, '/vio/odometry', self.vio_cb, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb, best_effort_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, reliable_qos)

        self.timer = self.create_timer(1.0, self.check)

    def vio_cb(self, msg):
        self.vio_time = self.get_clock().now()

    def local_cb(self, msg):
        self.local_time = self.get_clock().now()

    def state_cb(self, msg):
        self.state = msg
        self.state_time = self.get_clock().now()

    def fresh(self, t, max_age=2.0):
        if t is None:
            return False
        age = (self.get_clock().now() - t).nanoseconds / 1e9
        return age <= max_age

    def check(self):
        vio_ok = self.fresh(self.vio_time)
        local_ok = self.fresh(self.local_time)
        state_ok = self.fresh(self.state_time)

        if self.state is None:
            self.get_logger().warn('WAITING: /mavros/state not received')
            return

        connected_ok = self.state.connected
        guided_ok = self.state.guided and self.state.mode == 'GUIDED'
        disarmed_ok = not self.state.armed

        self.get_logger().info(
            f'vio={vio_ok} local_position={local_ok} state={state_ok} '
            f'connected={connected_ok} guided={guided_ok} disarmed={disarmed_ok} '
            f'mode={self.state.mode}'
        )

        if vio_ok and local_ok and state_ok and connected_ok and guided_ok and disarmed_ok:
            self.get_logger().info('PHASE2_HEALTH_PASS')
        else:
            self.get_logger().warn('PHASE2_HEALTH_NOT_READY')


def main(args=None):
    rclpy.init(args=args)
    node = Phase2HealthCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

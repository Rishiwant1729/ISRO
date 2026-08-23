import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OvToVio(Node):
    def __init__(self):
        super().__init__("ov_to_vio")
        self.declare_parameter("input_topic", "/ov_msckf/odomimu")
        self.declare_parameter("output_topic", "/vio/odometry")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(Odometry, output_topic, 20)
        self.sub = self.create_subscription(Odometry, input_topic, self.cb, 20)

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OvToVio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

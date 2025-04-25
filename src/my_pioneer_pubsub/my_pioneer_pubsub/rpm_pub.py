import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

RPM = 10

class RpmPublisher(Node):
    def __init__(self):
        super().__init__('rpm_pub_node')
        self.publisher_ = self.create_publisher(Int32, 'rpm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_rpm)
        self.i = 0

    def publish_rpm(self):
        msg = Int32()
        msg.data = int(RPM)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %d' % msg.data)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    rpm_pub_node = RpmPublisher()
    print('RPM publisher Node started')

    rclpy.spin(rpm_pub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rpm_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
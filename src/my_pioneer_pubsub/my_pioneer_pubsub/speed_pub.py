import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

# Pioneer 3-DX wheel diameter
WHEEL_DIAMETER = 195/1000 #milimeters to meters

class SpeedCalculator(Node):
    def __init__(self):
        super().__init__('speed_calc_node')
        self.subscription = self.create_subscription(
                            Int32,
                            'rpm',
                            self.listener_callback,
                            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(Float32, 'speed', 10)

    def listener_callback(self, rpm_msg):
        speed = rpm_msg.data * WHEEL_DIAMETER * 3.14259/60
        msg = Float32()
        msg.data = float(speed)
        self.publisher.publish(msg)
        self.get_logger().info(f'Receiving: {rpm_msg.data}rpm, Publishing: {msg.data:.4f}m/s')

def main(args=None):
    rclpy.init(args=args)

    speed_calc_node = SpeedCalculator()
    print("Speed calculator Node started")

    rclpy.spin(speed_calc_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speed_calc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
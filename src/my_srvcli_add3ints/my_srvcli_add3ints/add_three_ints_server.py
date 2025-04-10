# Import the necessary ROS 2 and Python packages
from my_custom_interfaces.srv import AddThreeInts  # import AddThreeInts.srv from my_custom_interface package
import rclpy
from rclpy.node import Node

# Define the service server class, inheriting from Node
class AddThreeIntsServer(Node):
    def __init__(self):
        # Initialize the node with the name 'add_three_ints_server'
        super().__init__('add_three_ints_server')
        # Create the service named 'add_three_ints' using the AddThreeInts service type
        # 'add_three_ints_callback' is the method that will be called when a request is received
        self.service = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)
        self.get_logger().info('Service is running..')

    # This function processes the service request.
    def add_three_ints_callback(self, request, response):     
        # The request contains three integers 'a', 'b' and 'c'.
        # The response will contain the sum of these integers.
        response.sum = request.a + request.b + request.c
        # Log the request and response
        self.get_logger().info(f'Receiving request: a={request.a}, b={request.b}, c={request.c} (sum: {response.sum})')
        # Return the response object, which contains the sum
        return response

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of your service server
    server = AddThreeIntsServer()
    # Spin the node so the callback function is called.
    # This will keep your service server alive and responsive to incoming service requests.
    rclpy.spin(server)
    # Shutdown the ROS client library before exiting the program
    rclpy.shutdown()

if __name__ == '__main__':
    main()
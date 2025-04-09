# Import the necessary ROS 2 and Python packages
from example_interfaces.srv import AddTwoInts # Make sure this matches the name of your .srv file and package
import rclpy
from rclpy.node import Node

# Define the service server class, inheriting from Node
class AddTwoIntsServer(Node):
    def __init__(self):
        # Initialize the node with the name 'add_two_ints_server'
        super().__init__('add_two_ints_server')
        # Create the service named 'add_two_ints' using the AddTwoInts service type
        # 'add_two_ints_callback' is the method that will be called when a request is received
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service is running..')

    # This function processes the service request.
    def add_two_ints_callback(self, request, response):     
        # The request contains two integers 'a' and 'b'.
        # The response will contain the sum of these integers.
        response.sum = request.a + request.b
        # Log the request and response
        self.get_logger().info(f'Receiving request: a={request.a}, b={request.b} (sum: {response.sum})')
        # Return the response object, which contains the sum
        return response

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of your service server
    server = AddTwoIntsServer()
    # Spin the node so the callback function is called.
    # This will keep your service server alive and responsive to incoming service requests.
    rclpy.spin(server)
    # Shutdown the ROS client library before exiting the program
    rclpy.shutdown()

if __name__ == '__main__':
    main()
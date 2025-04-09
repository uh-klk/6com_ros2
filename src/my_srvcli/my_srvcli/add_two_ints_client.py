# Import the necessary ROS 2 and Python packages
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

# Define the service client class, inheriting from Node
class AddTwoIntsClient(Node):
    def __init__(self):
        # Initialise the node witht he name 'add_two_ints_client'
        super().__init__('add_two_ints_client')
        # Create the client node with the same type (AddTwoInts) and name (add_two_ints} as service node
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # Check and wait for the sevice with the same type and name to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
    def send_request(self, a, b):
        # Create a new request with the integers to be added
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        # Async call to the service
        self.future = self.client.call_async(req)

def main():
    # Initialise the ROS client library
    rclpy.init()
    if len(sys.argv) == 3:
        try:
            # get the passed-in command-line arguments
            a = int(sys.argv[1])
            b = int(sys.argv[2])
        except ValueError:
            print('Usage: add_two_ints_client.py <int> <int>')
            sys.exit(1)
        
        # Create an instance of the service client
        client = AddTwoIntsClient()
        # Send a request with the provided integers
        client.send_request(a,b)

        #Wait fo the response from the service server
        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info('Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'Resulf of add_two_ints: {a} + {b} = {response.sum}')
                    break
    else:
        print('Usage: add_two_ints_client.py <int> <int>')
        sys.exit(1)
        
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
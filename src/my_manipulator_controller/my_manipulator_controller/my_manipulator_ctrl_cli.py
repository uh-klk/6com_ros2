# Import the necessary ROS2 and Python packages
import sys
import ast  # Used for safely converting string to list

from my_manipulator_interfaces.srv import ManipulatorControl  # Ensure this matches your .srv file and package name
from geometry_msgs.msg import Point

import rclpy
from rclpy.node import Node

# Define the service client class, inheriting from Node
class ManipulatorControllerClient(Node):
    def __init__(self):
        # Initialize the node with the name 'manipulator_controller_client'
        super().__init__('manipulator_controller_client')
        # Create the client for the 'manipulator_control' service
        self.client = self.create_client(ManipulatorControl, 'manipulator_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, command, position, position_list):
        # Create a new request with the integers to be added
        req = ManipulatorControl.Request()
        req.command = command
        req.position.x = float( position[0] )
        req.position.y = float( position[1] )
        req.position.z = float( position[2] )

        waypoints = []
        
        for i in range(len(position_list)):
            waypoint = Point()
            waypoint.x = float(position_list[i][0])
            waypoint.y = float(position_list[i][1])
            waypoint.z = float(position_list[i][2])
            waypoints.append(waypoint) 
        req.waypoints = waypoints
        
        # Example Service Call from the terminal
        # ros2 run my_manipulator_controller manipulator_controller_client "follow" [[1,1,1],[2,2,2]]

        #print(req, "\n")
        # my_manipulator_interfaces.srv.ManipulatorControl_Request(command='move', position=geometry_msgs.msg.Point(x=1.0, y=1.0, z=1.0), waypoints=[geometry_msgs.msg.Point(x=2.0, y=2.0, z=2.0), geometry_msgs.msg.Point(x=3.0, y=3.0, z=3.0)]')

        # Async call to the service
        self.future = self.client.call_async(req)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Ensure that the user has input two string and two lists of floting point numbers
    if len(sys.argv) >= 2:
        cmd = sys.argv[1]
        position = [0.0, 0.0, 0.0]
        position_list = [[0.0, 0.0, 0.0]]
        match cmd:
            case 'grasp': # request the gripper to grasp
                print(f"Request gripper to close.")

            case 'release': # request the gripper to release an object
                print(f"Request gripper to open.")
                
            case 'move': # request the gripper to move to a position
                if len(sys.argv) == 3:
                    try:
                        position = ast.literal_eval(sys.argv[2]) # use ast.literal_eval to evaluate the string containing a Python literal (like a list, dict, int, float, string, tuple, etc.) and returns the corresponding Python object
                        #print(validate_coordinate(position))
                    except ValueError:
                        print('Type issue, Usage: ... manipulator_controller_client move [<float>,<float>,<float>]')
                        sys.exit(1)
                else:
                    print('Missing argument, Usage: ... manipulator_controller_client move [<float>,<float>,<float>]')
                    sys.exit(1)
                print(f"Request gripper moving to position ({position[0]}, {position[1]}, {position[2]}).")

            case 'follow': # request the gripper to follow a path (i.e., waypoints)
                if len(sys.argv) == 3:
                    try:
                        position_list  = ast.literal_eval(sys.argv[2]) # use ast.literal_eval to evaluate the string containing a Python literal (like a list, dict, int, float, string, tuple, etc.) and returns the corresponding Python object
                    except ValueError:
                        print('Type issue, Usage: ... manipulator_controller_client follow [[<float>,<float>,<float>],[<float>,<float>,<float>],...]')
                        sys.exit(1)
                else:
                    print('Missing argument, Usage: ... manipulator_controller_client follow [[<float>,<float>,<float>],[<float>,<float>,<float>],...]')
                    sys.exit(1)
                print("Request gripper to follow the following waypoints...")
                for i in range(len(position_list)):
                    print(f"\tgoing to waypoint {i} ({position_list[i][0]}, {position_list[i][1]}, {position_list[i][2]}) ...")

            case   _:
                status = 'failed'
                message = 'Unsupported action! Supported commands are move, follow, grasp or release.'
                print(message) 
                sys.exit(1)

        # Create an instance of the service client
        client = ManipulatorControllerClient()
        # Send a request with the provided integers
        client.send_request(cmd, position, position_list)
        # Wait for the response from the service server
        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info('Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'\nResult of manipulator_control:{response.status}')
                    client.get_logger().info(f'\n{response.message}\n')
                break
    else:
        print('Usage: ... manipulator_controller_client grasp   <str>')
        print('Usage: ... manipulator_controller_client release <str>')
        print('Usage: ... manipulator_controller_client move    [<float>,<float>,<float>] ') #no space between elements nor brackets
        print('Usage: ... manipulator_controller_client follow  [[<float>,<float>,<float>],...]') #no space between elements nor brackets
        sys.exit(1)

    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()


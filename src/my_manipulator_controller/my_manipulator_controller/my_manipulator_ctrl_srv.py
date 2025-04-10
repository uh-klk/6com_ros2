# Import the necessary ROS 2 and Python packages
from my_manipulator_interfaces.srv import ManipulatorControl  # Make sure this matches the name of your .srv file and package
import rclpy
from rclpy.node import Node

# Define the service server class, inheriting from Node
class ManipulatorControllerServer(Node):
    def __init__(self):
        # Initialize the node with the name 'manipulator_controller_server'
        super().__init__('manipulator_controller_server')
        # Create the service named 'manipulator_control' using the ManipulatorControl service type
        # 'manipulator_control_callback' is the method that will be called when a request is received
        self.service = self.create_service(ManipulatorControl, 'manipulator_control', self.manipulator_control_callback)

    def manipulator_control_callback(self, request, response):
        # This function processes the service request.
        # The request contains
        # Perform the addition
        #request.command
        #request.position.x, #request.position.y, #request.position.z
        #request.waypoints[].x, request.waypoints[].y, request.waypoints[].z 
        #request.obj_id
        
        # Example Service Call from the terminal
        #   ros2 service call /manipulator_control my_manipulator_interfaces/srv/ManipulatorControl "{command: 'move', position: {x: 1.0, y: 1.0, z: 1.0}, waypoints: [{x: 2.0, y: 2.0, z: 2.0}, {x: 3.0, y: 3.0, z: 3.0}], obj_id: 'apple_1'}"
        print(request)
        # my_manipulator_interfaces.srv.ManipulatorControl_Request(command='move', position=geometry_msgs.msg.Point(x=1.0, y=1.0, z=1.0), waypoints=[geometry_msgs.msg.Point(x=2.0, y=2.0, z=2.0), geometry_msgs.msg.Point(x=3.0, y=3.0, z=3.0)], obj_id='apple_1')

        match request.command:
            case 'move': # request the gripper to move to request.position 
                print(f"Moving to position ({request.position.x}, {request.position.y}, {request.position.z}).")
                print(f"Reached ({request.position.x}, {request.position.y}, {request.position.z}).\n")
                status = 'success'
                message = 'Reached ' + str(request.position)

            case 'grasp': # request the gripper to grasp
                print("Gripper closing ...")
                print("Gripper state: closed.\n")
                status = 'success'
                message = request.obj_id + ' has been grasped'

            case 'release': # request the gripper to release an object
                print("Gripper openining ...")
                print("Gripper state: open.\n")
                status = 'success'
                message = request.obj_id + ' has been released' 
            
            case 'follow': # request the gripper to follow a path (i.e., waypoints)
                print("Following waypoints")
                for i in range(len(request.waypoints)):
                    print(f"going to waypoint {i} ({request.waypoints[i].x}, {request.waypoints[i].y}, {request.waypoints[i].z}) ...")

                print(f"Arrived at final waypoint ({request.waypoints[i].x}, {request.waypoints[i].y}, {request.waypoints[i].z}).\n")

                status = 'success'
                message = 'Successfully following ' + str(request.waypoints) + ' path'

            case   _:
                status = 'failed'
                message = 'Unsupported action! Supported commands are move,f ollow, grasp or release.'
                print(message)

        response.status = status
        response.message = message

        # Log the request and response
        #self.get_logger().info(f'Receiving request: a={request.a}, b={request.c}, c={request.c} (sum: {response.sum})')
        
        # Return the response object, which now contains the sum
        return response

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of your service server
    server = ManipulatorControllerServer()

    # Spin the node so the callback function is called.
    # This will keep your service server alive and responsive to incoming service requests.
    rclpy.spin(server)

    # Shutdown the ROS client library before exiting the program
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
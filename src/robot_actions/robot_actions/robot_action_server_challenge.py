# robot_action_server.py
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from my_custom_interfaces.action import MoveTo

class RobotActionServer(Node):
    def __init__(self):
        super().__init__('robot_action_server') 
        self._action_server = ActionServer(self, MoveTo, 'move_to', self.execute_callback, cancel_callback=self.cancel_callback)

    # See example https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveTo.Feedback()
        feedback_msg.current_distance_travelled = 0.0

        while feedback_msg.current_distance_travelled < goal_handle.request.target_distance:
            if goal_handle.is_cancel_requested: #add for goal cancel
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = MoveTo.Result()
                result.total_distance_travelled = feedback_msg.current_distance_travelled
                return result

            feedback_msg.current_distance_travelled +=1.0
            self.get_logger().info(f'Feedback:{feedback_msg.current_distance_travelled}m')
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=1)

        goal_handle.succeed()

        result = MoveTo.Result()
        result.total_distance_travelled = feedback_msg.current_distance_travelled
        return result

def main(args=None):
    rclpy.init(args=args)
    robot_action_server = RobotActionServer()
    print("Action Server Running...")
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_action_server)
    except KeyboardInterrupt:
        robot_action_server._action_server.destroy()
        robot_action_server.destroy_node()

if __name__ == '__main__':
    main()
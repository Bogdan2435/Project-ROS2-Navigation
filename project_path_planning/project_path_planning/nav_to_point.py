import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_point')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.declare_parameter('spot_name', rclpy.Parameter.Type.STRING)

        self.spot_name = self.get_parameter(name="spot_name").value
        self.get_logger().info(f'Received spot_name: {self.spot_name}')

        
        if self.spot_name in ['corner1', 'corner2', 'pedestrian', 's1', 's2']:
            # txt = self.spot_name + '.position.position_x'
            # self.get_logger().info(f'init: {txt}')
            self.spot_name = str(self.spot_name)
            self.declare_parameter(self.spot_name + '.position.position_x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.spot_name + '.position.position_y', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.spot_name + '.orientation.orientation_z', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.spot_name + '.orientation.orientation_w', rclpy.Parameter.Type.DOUBLE)

            # self.declare_parameter(self.spot_name + '.position.position_x', rclpy.Parameter.Type.DOUBLE)
            # self.declare_parameter(self.spot_name + '.position.position_y', '')
            # self.declare_parameter(self.spot_name + '.orientation.orientation_z', '')
            # self.declare_parameter(self.spot_name + '.orientation.orientation_w', '')

            # self.x = self.get_parameter(name='s1.position.position_x').value
            # self.x = self.get_parameter(name=self.spot_name + '.position.position_x').value
            self.x = self.get_parameter(self.spot_name + '.position.position_x').value
            self.y = self.get_parameter(name=self.spot_name + '.position.position_y').value
            self.z = self.get_parameter(name=self.spot_name + '.orientation.orientation_z').value
            self.w = self.get_parameter(name=self.spot_name + '.orientation.orientation_w').value

            self.get_logger().info(f'Received X: {self.x}')
            self.get_logger().info(f'Received X: {self.y}')
            self.get_logger().info(f'Received X: {self.z}')
            self.get_logger().info(f'Received X: {self.w}')

        self.send_goal()

    def send_goal(self):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        # goal_pose.pose.position.x = self.x
        # goal_pose.pose.position.y = self.y
        # goal_pose.pose.orientation.z = self.z
        # goal_pose.pose.orientation.w = self.w

        goal_pose.pose.pose.position.x = self.x
        goal_pose.pose.pose.position.y = self.y
        goal_pose.pose.pose.orientation.z = self.z
        goal_pose.pose.pose.orientation.w = self.w

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

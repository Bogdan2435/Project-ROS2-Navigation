import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from custom_interfaces.srv import MyServiceMessage
import os

class SpotRecorder(Node):

    def __init__(self):
        super().__init__('spot_recorder')
        self.group1 = ReentrantCallbackGroup()

        self.srv = self.create_service(MyServiceMessage, 'spot_record', self.service_callback, callback_group=self.group1)
        self.subscriber_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group1)

        # self.txt_path = os.path.join(get_package_share_directory('project_localization'), 'txt/spots.txt')
        self.txt_path = "/home/user/ros2_ws/spots.txt"
        self.spots = [] # list of dictionaries
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.orient_z = 0.0
        self.orient_w = 0.0
        self.end = False

    def service_callback(self, request, response):

        if request.label == 'end':
            # try:
            self.get_logger().info('Path : "%s"' % str(self.txt_path))

            with open(self.txt_path, 'w') as f:
                for spot in self.spots:
                    txt = 'label: ' + str(spot['label']) + '\n'
                    f.write(txt)

                    txt = '  position_x: ' + str(spot['position_x']) + '\n'
                    f.write(txt)

                    txt = '  position_y: ' + str(spot['position_y']) + '\n'
                    f.write(txt)

                    txt = '  orientation_z: ' + str(spot['orientation_z']) + '\n'
                    f.write(txt)

                    txt = '  orientation_w: ' + str(spot['orientation_w']) + '\n'
                    f.write(txt)
            response.navigation_successfull = True
            self.end = True
            response.message = "Spots saved to txt file"
            # except:
            #     response.navigation_successfull = False
            #     self.end = True
            #     response.message = "Spots NOT saved to txt file"

        else:
            spot = {
                'label': request.label,
                'position_x': self.pose_x,
                'position_y': self.pose_y,
                'orientation_z': self.orient_z,
                'orientation_w': self.orient_w,
            }
            self.spots.append(spot)
            response.navigation_successfull = True
            response.message = "Spot " + request.label + " registered."

        return response

    def amcl_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.orient_z = msg.pose.pose.orientation.z
        self.orient_w = msg.pose.pose.orientation.w
        self.get_logger().info('Data received : "%s"' % str(self.pose_x))

def main(args=None):
    rclpy.init(args=args)
    service = SpotRecorder() 
    while not service.end:
        rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
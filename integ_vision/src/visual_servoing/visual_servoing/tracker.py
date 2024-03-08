import rclpy
from rclpy.node import Node
from integ_my_robot_interfaces.msg import CenterAndArea
from std_msgs.msg import Float64
import numpy as np
import math
from sensor_msgs.msg import JointState

class Tracker(Node):
    def __init__(self):
        super().__init__('Tracker')

        #a joint state subscriber
        self.joint_subscriber = self.create_subscription(JointState, '/scara/joint_states', self.joint_states_callback, 10)

        # a subscriber to the DetectedFeature topic
        self.feature_subscriber = self.create_subscription(CenterAndArea, 'center_and_area_detected', self.listener_callback, 10)
        
        # publisher of the joint1 position command
        self.joint1_publisher_ = self.create_publisher(Float64, '/scara/joint_1_cmd_pos', 10)
        # publisher of the joint2 position command
        self.joint2_publisher_ = self.create_publisher(Float64, '/scara/joint_2_cmd_pos', 10)

    def joint_states_callback(self, msg):
        self.q1 = msg.position[1] # opposite sign to the joint state because the joint state is published in the opposite direction
        self.q2 = msg.position[2]

    def listener_callback(self, msg):

        joint1_command = Float64()
        joint2_command = Float64()

        if msg.detected == True :

            error_x = msg.center.x-320 # 320 is the center of the image
            error_y = msg.center.y-200 # 200 is the center of the image

            vertical_fov = 2.094 # the vertical field of view of the camera in radians
            horizontal_fov = 2.094 # the horizontal field of view of the camera in radians

            distance_to_target = 0.11 # the distance from the camera to the target in meters

            image_width_meters = 2*distance_to_target*math.tan(horizontal_fov/2)
            image_height_meters = 2*distance_to_target*math.tan(vertical_fov/2)

            error_vector = [-error_y/400*image_height_meters,-error_x/640*image_width_meters] # convert the error vector from pixels to meters and takes into account the orientation of the camera relative to the end effector
            
            desired_position = [-1 * component for component in error_vector]

            l1 = 0.28002 # the length of the first link
            l2 = 0.28002 # the length of the second link

            # solve inverse kinematics to get the joint angles that result in a movement of -error_vector
            try :
                c2 = (desired_position[0]**2 + desired_position[1]**2 - l1**2 - l2**2)/(2*l1*l2) 
                s2 = math.sqrt(1 - c2**2)

                c1 = ((l1+l2*c2)*desired_position[0] + l2*s2*desired_position[1])/(desired_position[0]**2 + desired_position[1]**2)
                s1 = ((l1+l2*c2)*desired_position[1] - l2*s2*desired_position[0])/(desired_position[0]**2 + desired_position[1]**2)

                theta1 = math.atan2(s1, c1)
                theta2 = math.atan2(s2, c2)

            #convert the joint differences angles between q1 and theta 1 to degrees and print them DEBUGGING
                joint1_diff = (theta1 - self.q1) * 180 / math.pi
                joint2_diff = (theta2 - self.q2) * 180 / math.pi
                self.get_logger().info(f'joint1 diff: {joint1_diff}, joint2 diff: {joint2_diff}')

                joint1_command.data = joint1_diff
                joint2_command.data = joint2_diff

                self.joint1_publisher_.publish(joint1_command)
                #self.joint2_publisher_.publish(joint2_command)
            except Exception as e:
                self.get_logger().error(f'Error in inverse kinematics: {e}')
                return

        else:
            self.get_logger().info('No feature detected')


def main(args=None):
    rclpy.init(args=args)
    tracker = Tracker()
    # Add your main code here
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
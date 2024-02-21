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
        self.joint1_publisher_ = self.create_publisher(Float64, '/scara/joint_1_cmd_vel', 10)
        # publisher of the joint2 position command
        self.joint2_publisher_ = self.create_publisher(Float64, '/scara/joint_2_cmd_vel', 10)

        self.latest_joint1_command = Float64()
        self.latest_joint2_command = Float64()

    def joint_states_callback(self, msg):
        self.q1 = msg.position[1]
        self.q2 = msg.position[2]

    def listener_callback(self, msg):

        joint1_command = Float64()
        joint2_command = Float64()

        if msg.detected == True : # if a feature is detected

            try : # implement the command law here

                error_x = abs(320 - msg.center.x) # 320 is the center of the image
                error_y = abs(200 - msg.center.y) # 200 is the center of the image

                vertical_fov = 2.094 # the vertical field of view of the camera in radians
                horizontal_fov = 2.094 # the horizontal field of view of the camera in radians

                distance_to_target = 0.11 # the distance from the camera to the target in meters

                image_width_meters = 2*distance_to_target*math.tan(horizontal_fov/2)
                image_height_meters = 2*distance_to_target*math.tan(vertical_fov/2)

                alpha_x = 640/image_width_meters # the conversion factor from pixels to meters in the x direction
                alpha_y = 400/image_height_meters # the conversion factor from pixels to meters in the y direction

                l1 = 0.28002 # the length of the first link
                l2 = 0.28002 # the length of the second link

                # rotation matrix of -pi/2 around x and z
                W = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])

                J = np.array([[-l1*np.sin(self.q1)-l2*np.sin(self.q1+self.q2), -l2*np.sin(self.q1+self.q2),0],
                            [l1*np.cos(self.q1)+l2*np.cos(self.q1+self.q2), l2*np.cos(self.q1+self.q2),0],
                            [0, 0, 1]])  
                
                Pixel2MeterMatrix = np.array([[alpha_x, 0], [0, alpha_y]])

                #L2d = np.array([[-1/distance_to_target,0 , pow(error_x/(alpha_x),2)/distance_to_target], [0, -1/distance_to_target , pow(error_y/(alpha_y),2)/distance_to_target]])
                L2d = np.eye(2,3)

                #multiply Pixel2MeterMatrix and L2d and W and J
                Js = np.dot(np.dot(Pixel2MeterMatrix, L2d), np.dot(W, J))
                Js_pseudo_inv = np.linalg.pinv(Js)    

                coef = 0.1
                q_dot = np.dot(Js_pseudo_inv, [-coef*error_x, -coef*error_y])

                joint1_command.data = q_dot[0]
                joint2_command.data = q_dot[1]

                #print the commands in degrees
                self.get_logger().info(f'Joint1 Command: {math.degrees(joint1_command.data)}')
                self.get_logger().info(f'Joint2 Command: {math.degrees(joint2_command.data)}')
                self.joint1_publisher_.publish(joint1_command)
                self.joint2_publisher_.publish(joint2_command)

                self.latest_joint1_command = joint1_command
                self.latest_joint2_command = joint2_command

            except Exception as e:
                self.get_logger().error(f'calculating command: {e}')
                return

        else: #if no feature is detected publish the latest joint commands again
            self.joint1_publisher_.publish(self.latest_joint1_command)
            self.joint2_publisher_.publish(self.latest_joint2_command)
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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from integ_my_robot_interfaces.msg import CenterAndArea

class FeatureDetector(Node):
    def __init__(self):
        super().__init__('FeatureDetector')
        ## a subscriber to the camera topic
        self.subscription = self.create_subscription(Image, 'scara/image', self.listener_callback, 10)
        # a publisher of a custom message type that contains a point and and integer
        self.publisher_ = self.create_publisher(CenterAndArea, 'center_and_area_detected', 10)
        self.cv_bridge = CvBridge() # Create a CvBridge object to convert the image message to OpenCV format

    def listener_callback(self, msg):
        #TODO analyse the image using openCV and counts the number of blue pixels  and the center of the blue pixels then publish the center and the number of blue pixels
        
        # Convert the image message to OpenCV format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting image from ros2 to openCV format: {e}')
            return

        converted_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        #convert from RGB to HSV
        
        converted_image_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #filter blue color using HSV format
        lower_blue_hsv = np.array([0, 50, 50])
        upper_blue_hsv = np.array([255, 255, 255])

        window_name = 'Image in HSV format'
        cv2.imshow(window_name, converted_image_HSV) 
        cv2.waitKey(3)

        # Define the lower and upper bounds for the blue color in BGR (OpenCV) format
        #lower_blue = np.array([100, 0, 0])
        #upper_blue = np.array([255, 100, 100])

        # Threshold the image to get only the blue pixels using HSV
        mask = cv2.inRange(converted_image_HSV, lower_blue_hsv, upper_blue_hsv)

        cv2.imshow('Blue Mask', mask)
        cv2.waitKey(3)

        # Count the number of blue pixels
        blue_pixel_count = cv2.countNonZero(mask)

        # Calculate the center of the blue pixels
        M = cv2.moments(mask)

        center_and_area = CenterAndArea()

        try :
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

            self.get_logger().info(f'Center: ({center_x}, {center_y}), Blue pixels: {blue_pixel_count}')
            center_and_area.center.x = float(center_x)
            center_and_area.center.y = float(center_y)
            center_and_area.area = blue_pixel_count
            center_and_area.detected = True
            self.publisher_.publish(center_and_area)
        except Exception as e:
            center_and_area.detected = False
            self.get_logger().error(f'Error processing the mask, likely a division by 0 calculating the center: {e}')
            self.publisher_.publish(center_and_area)




def main(args=None):
    rclpy.init(args=args)
    detector = FeatureDetector()
    # Add your main code here
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
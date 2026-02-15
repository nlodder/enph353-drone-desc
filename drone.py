#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge, CvBridgeError

class image_imu_to_wrench:

    SLICE_HEIGHT = 40
    DARKNESS_THRESHOLD = 20
    ROBOT_SPEED = 0.4 # m/s

    def __init__(self):
        """
        Initializes the image_imu_to_wrench node.
        """
        rospy.init_node('image_imu_to_wrench', anonymous=False)
        
        self.bridge = CvBridge()
        self.last_x = None  # Persistent state
        self.imu = GazeboRosImuSensor()
        
        # rostopic subscriptions
        self.image_sub = rospy.Subscriber("drone/camera1/image_raw", Image, self.image_callback)
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=1)

        # rostopic publications
        self.rotor_fl_thrust_pub = rospy.Publisher("rotor_fl_thrust", Wrench, queue_size=1)
        self.rotor_fr_thrust_pub = rospy.Publisher("rotor_fr_thrust", Wrench, queue_size=1)
        self.rotor_rl_thrust_pub = rospy.Publisher("rotor_rl_thrust", Wrench, queue_size=1)
        self.rotor_rr_thrust_pub = rospy.Publisher("rotor_rr_thrust", Wrench, queue_size=1)
    
    def imu_callback(self, data):
        """
        Assigns new imu data to class imu 
        
        :param self: Description
        :param data: Description
        """
        #TODO fill this in
    
    def image_callback(self, data):
        """
        Callback function for image subscriber. Runs on receiving an image.
        
        :param data: Image message from robot camera topic
        """
        try:
            bgr8_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.rotor_wrench_dict = {
                "fr" : None,
                "fl" : None,
                "rr" : None,
                "rl" : None
            }
            self.rotor_wrench = self.sens_to_wrench(bgr8_image)
            self.rotor_fl_thrust_pub.publish(self.rotor_wrench_dict["fl"])
            self.rotor_fr_thrust_pub.publish(self.rotor_wrench_dict["fr"])
            self.rotor_rr_thrust_pub.publish(self.rotor_wrench_dict["rr"])
            self.rotor_rl_thrust_pub.publish(self.rotor_wrench_dict["rl"])
        except CvBridgeError as e:
            print(e)
        return
    
    def image_to_cmd_vel(self, bgr8_image):
        """
        Converts BGR8 image to Twist command for line following.

        :param bgr8_image: Input image in BGR8 format from robot camera
        :return: Twist message for robot movement
        """
        # extract region of interest (roi)
        cap_height, cap_width, _ = bgr8_image.shape
        if self.last_x is None: # if line never seen
            last_x = cap_width // 2
        y_start = cap_height - self.SLICE_HEIGHT
        roi = bgr8_image[y_start:cap_height, :]
        hsv_roi, avg_v = self.preprocess_slice(roi)
        
        # get line mask
        mask = self.get_line_mask(hsv_roi, avg_v)

        # update state
        current_x = self.find_cent_x_from_mask(mask)
        if current_x is not None:
            self.last_x = current_x

        # create and return Wrench message
        wrench_msg = Wrench()
        error_x = self.last_x - (cap_width // 2)
        wrench_msg.force.z = -float(error_x) / (cap_width // 2)
        wrench_msg.torque.x = self.ROBOT_SPEED * (1 - abs(wrench_msg.angular.z))
        return wrench_msg
    
    def find_cent_x_from_mask(self, mask):
        """
        Finds the centroid x-coordinate from a binary mask.

        :param mask: Binary image mask
        :return: x-coordinate of the centroid or None if not found"""
        moment = cv.moments(mask)
        # check if mask has white pixels to avoid division by zero
        if moment["m00"] != 0:
            cent_x = int(moment["m10"] / moment["m00"])
            return cent_x
        else:
            return None
    
    def get_line_mask(self, hsv_slice, avg_v):
        """
        Creates a binary mask for the line in the HSV slice. 

        :param hsv_slice: Image in HSV format
        :param avg_v: Average V channel (brightness) value 
        :return: Binary mask image
        """
        upper_v = int(np.clip(avg_v - self.DARKNESS_THRESHOLD, 0, 255))
        lower_bound = np.array([0, 0, 0], dtype=np.uint8)
        upper_bound = np.array([179, 255, upper_v], dtype=np.uint8)
        return cv.inRange(hsv_slice, lower_bound, upper_bound)

    def preprocess_slice(self, slice_image):
        """
        Converts BGR to HSV and returns V channel average brightness.

        :param slice_image: Image in BGR format
        :return: HSV image and average brightness of V channel
        """
        hsv_image = cv.cvtColor(slice_image, cv.COLOR_BGR2HSV)
        avg_brightness = np.mean(hsv_image[:, :, 2])
        return hsv_image, avg_brightness

def main():
    node = image_imu_to_wrench()
    try:
        # keep node running until interrupted
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
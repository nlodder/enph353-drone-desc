#!/usr/bin/env python3
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DroneTestNode:
    def __init__(self):
        rospy.init_node('drone_test_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic defined in your Gazebo plugin
        self.image_sub = rospy.Subscriber("drone/camera1/image_raw", Image, self.image_callback)
        rospy.loginfo("Drone Test Node Initialized. Waiting for images...")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Show the image in a window
            cv.imshow("Drone Camera Feed", cv_image)
            cv.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

def main():
    test_node = DroneTestNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
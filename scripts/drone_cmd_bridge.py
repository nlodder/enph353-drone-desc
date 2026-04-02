#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64

class DroneCmdBridge:
    def __init__(self):
        rospy.init_node('drone_cmd_bridge')
        self.UPDATE_HZ = 60

        # latch last command
        self.current_twist = Twist()

        # subscribe to drone's cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        # these will become dronex/planar_cmd and dronex/vert_cmd from launch namespacing
        self.planar_pub = rospy.Publisher("planar_cmd", Twist, queue_size=1)
        self.vertical_pub = rospy.Publisher("vert_cmd", Float64, queue_size=1)
    
    def cmd_callback(self, msg):
        self.current_twist = msg
    
    def run(self):
        # rate of cmd updates to planar and vertical movement plugins
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self.planar_pub.publish(self.current_twist)
            self.vertical_pub.publish(Float64(self.current_twist.linear.z))
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = DroneCmdBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
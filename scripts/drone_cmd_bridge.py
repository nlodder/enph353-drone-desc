#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64

class DroneCmdBridge:
    def __init__(self):
        rospy.init_node('drone_cmd_bridge')
        self.UPDATE_HZ = 60

        # latch last command
        self.current_wrench = Wrench()

        # subscribe to drone's cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        # these will become dronex/planar_cmd and dronex/vert_cmd from launch namespacing
        self.mov_pub = rospy.Publisher("cmd_force", Wrench, queue_size=1)
    
    def cmd_callback(self, msg):
        self.current_wrench = msg
    
    def run(self):
        # rate of cmd updates to planar and vertical movement plugins
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self.mov_pub.publish(self.current_wrench)
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = DroneCmdBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
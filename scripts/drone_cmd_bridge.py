import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DroneCmdBridge:
    def __init__(self):
        rospy.init_node('drone_cmd_bridge')
        self.UPDATE_HZ = 100

        # latch last command
        self.current_twist = Twist()

        # subscribe to drone's cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        # these will become dronex/planar_cmd and dronex/vert_cmd from launch namespacing
        rospy.Publisher("planar_cmd", Twist, queue_node=1)
        rospy.Publisher("vert_cmd", Float64, queue_node=1)
    
    def cmd_callback(self, msg):
        self.current_twist = msg
    
    def run(self):
        # rate of cmd updates to planar and vertical movement plugins
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self.planar_pub.publish(self.current_twist)
            self.vertical_pub.publish(self.current_twist.linear.z)
            rate.sleep()
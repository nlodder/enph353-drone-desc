#!/usr/bin/env python3
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64
import rospy
from gazebo_msgs.srv import GetPhysicsProperties
import xml.etree.ElementTree as ET #to parse urdf for mass data

class DroneVertMv:
    def __init__(self):
        self.ns = rospy.get_namespace().strip('/')
        gravity = self.get_gazebo_gravity()
        total_mass = self.get_total_mass()
        self.HOVER_FORCE = gravity * total_mass
        self.FORCE_SCALE = 100

        # subscribe to drone's vert_cmd topic
        rospy.Subscriber("vert_cmd", Float64, self.apply_lift)

    def apply_lift(self, linear_vel_z):
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        try:
            apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            
            wrench = Wrench()
            wrench.force = Vector3(0, 0, self.HOVER_FORCE + linear_vel_z.data * self.FORCE_SCALE)
            
            # Apply it to the body link
            rospy.loginfo(f"Vertical Force: {wrench.force}, Hover Force: {self.HOVER_FORCE}")
            apply_wrench(body_name=f"{self.ns}::link_drone_body", wrench=wrench, duration=rospy.Duration(0.1))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_gazebo_gravity(self):
        rospy.wait_for_service('/gazebo/get_physics_properties')
        try:
            physics_proxy = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
            results = physics_proxy()
            # gravity is a Vector3; we take the magnitude of z (usually -9.8)
            return abs(results.gravity.z)
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return 9.81 # Default fallback
    
    def get_total_mass(self):
        # Using a relative param name if it's inside the namespace
        # or "/robot_description" if it's global.
        try:
            xml_string = rospy.get_param("robot_description")
            root = ET.fromstring(xml_string)
            total_mass = 0.0
            for mass_tag in root.findall(".//mass"):
                total_mass += float(mass_tag.get('value'))
            
            rospy.loginfo(f"calculated total mass: {total_mass} kg")
            return total_mass
    
        except Exception as e:
            rospy.logerr(f"Could not parse URDF: {e}")
            return 0.5 # kg, Default fallback mass

if __name__ == "__main__":
    rospy.init_node("drone_vert_mover")
    mover = DroneVertMv()
    rospy.spin()
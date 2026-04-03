#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import ApplyBodyWrench
import xml.etree.ElementTree as ET #to parse urdf for mass data 
import tf
import math

class DroneCmdBridge:
    def __init__(self):
        """
            Initializes the DroneCmdBridge node, sets up PID controllers for stabilization and altitude control, and subscribes to necessary topics.
            - Initializes ROS node and sets up parameters for the target body, hover force, and PID controllers for roll, pitch, and altitude stabilization.
            - Subscribes to "cmd_vel" for velocity commands, "imu" for orientation data, and "altitude" for current altitude updates.
            - Prepares to apply wrenches to the drone in Gazebo based on the received commands and sensor data.
        """
        rospy.init_node('drone_cmd_bridge')
        
        self.ns = rospy.get_namespace().strip('/')
        self.target_body = f"{self.ns}::link_drone_body"

        gravity = self.get_gazebo_gravity()
        total_mass = self.get_total_mass()
        self.HOVER_FORCE = gravity * total_mass
        self.FORCE_SCALE = 1.0
        
        # PID for angular stabilization
        self.pitch_PID = StabilizePIDController(kp=0.7, ki=0.4, kd=0.2)
        self.roll_PID = StabilizePIDController(kp=0.7, ki=0.4, kd=0.2)
        self.target_euler = [0, 0] # target roll and pitch in radians

        self.elev_PID = ElevPIDController(kp=1.0, ki=0.15, kd=0.2)
        self.desired_z = 1.0 # desired altitude in meters
        self.current_z = 1.0 # current altitude in meters, updated from Gazebo

        self.UPDATE_HZ = 30
        self.DUR_BUFFER = rospy.Duration(1.5 / self.UPDATE_HZ) # force applied for update period

        self.current_wrench = Wrench()
        self.current_wrench.force.z = self.HOVER_FORCE
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        # subscribe to drone's cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("altitude", Float64, self.altitude_callback)
        
    
    def vel_callback(self, msg):
        """
            Updates the current wrench based on the incoming cmd_vel message.
            - msg: geometry_msgs/Twist message containing desired linear and angular velocities.
            - The linear x and y components of the cmd_vel are scaled and set as forces in the current wrench.
            - The linear z component of the cmd_vel is used to adjust the desired altitude (self.desired_z)
        """
        self.desired_z += msg.linear.z  # Update desired altitude
        self.desired_z = max(0.1, self.desired_z)  # Prevent going below ground level

        self.current_wrench.force.x = msg.linear.x * self.FORCE_SCALE
        self.current_wrench.force.y = msg.linear.y * self.FORCE_SCALE

    def imu_callback(self, msg):
        """Updates the current orientation of the drone and computes the necessary torques to stabilize roll and pitch angles."""
        # get orientation from the IMU message as a quaternion
        # this is the orientation of the drone in the world frame
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion) # convert to radians

        torque_roll = self.roll_PID.update(self.target_euler[0], roll, 1.0 / self.UPDATE_HZ)
        torque_pitch = self.pitch_PID.update(self.target_euler[1], pitch, 1.0 / self.UPDATE_HZ)

        # PD for angular stabilization
        self.current_wrench.torque.x = torque_roll
        self.current_wrench.torque.y = torque_pitch
        self.current_wrench.torque.z = 0  # No stabilization needed for yaw
        # self.show_pattern(quaternion, euler, error_roll, error_pitch)

    def altitude_callback(self, msg):
        """Updates the current altitude of the drone (self.current_z)"""
        self.current_z = msg.data

    def run(self):
        """
            Main loop to continuously apply the current wrench to the drone in Gazebo.
            - Uses a ROS rate to control the update frequency.
            - Before applying the wrench, it calls update_current_wrench_z() to adjust the vertical force based on altitude errors using the elevation PID controller.
            - Applies the wrench using the /gazebo/apply_body_wrench service, specifying the target body and reference frame.
            - Catches any service exceptions and logs them.
        """
        rate = rospy.Rate(self.UPDATE_HZ)


        while not rospy.is_shutdown():
            # update vertical force based on altitude error using PID
            self.update_current_wrench_z()

            rospy.wait_for_service('/gazebo/apply_body_wrench')
            
            try:
                self.apply_wrench(body_name=self.target_body,
                                  reference_frame=self.target_body, # so that force applied in drone frame
                                  wrench=self.current_wrench,
                                  duration=self.DUR_BUFFER)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

            # self.force_pub.publish(self.current_wrench)
            rate.sleep()
    
    def update_current_wrench_z(self):
        """
            Update the vertical force component of the current wrench based on the altitude error using the elevation PID controller.
            Modifies self.current_wrench.force.z to be the hover force plus the PID output needed to correct altitude errors.
        """
        self.z_needed = self.elev_PID.update(self.desired_z, self.current_z, 1.0 / self.UPDATE_HZ)
        self.current_wrench.force.z = self.HOVER_FORCE + self.z_needed
        return
    
    def get_gazebo_gravity(self):
        """
            Calls the /gazebo/get_physics_properties service to retrieve the gravity vector and returns the magnitude of the z component as a positive value.
            If the service call fails, it logs an error and returns a default gravity value of 9.81 m/s^2.
        """
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
        """Parses the URDF to calculate the total mass of the drone by summing all <mass> tags. This is used to calculate the hover force needed to counteract gravity.
         - Uses rospy.get_param to retrieve the robot_description parameter which contains the URDF XML string
         - Parses the XML to find all <mass> tags and sums their 'value' attributes
         - Returns the total mass as a float. If parsing fails, returns a default mass of 0.5 kg.
        """
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
    
    def show_pattern(self, quat, euler, error_roll, error_pitch):
        """Utility function to print the current orientation and stabilization errors in a readable format directly in terminal."""
        pattern = f"Quat: [{quat[0]:.2f}, {quat[1]:.2f}, {quat[2]:.2f}, {quat[3]:.2f}] | "
        pattern += f"Euler: [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}] | "
        pattern += f"Error Roll: {error_roll:.2f} | Error Pitch: {error_pitch:.2f}"
        rospy.loginfo(pattern)

class StabilizePIDController:
    """Simple PID controller for stabilizing roll and pitch angles."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
    
    def update(self, target_rad, current_rad, dt):
        """
        Simple PID controller to stabilize the drone's orientation.
         - target_rad: desired angle in radians (roll or pitch)
         - current_rad: current angle in radians (roll or pitch)
         - dt: time step in seconds
         Returns the control output (torque) to apply.
        """
        # if the roll/pitch is positive, apply negative torque to stabilize, and vice versa
        error = target_rad - current_rad
        # normalize error to be within [-pi, pi]
        error = math.atan2(math.sin(error), math.cos(error))
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class ElevPIDController:
    """Simple PID controller for stabilizing altitude."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
    
    def update(self, target_z, current_z, dt):
        """
        Simple PID controller to stabilize the drone's altitude.
         - target_z: desired altitude in meters
         - current_z: current altitude in meters
         - dt: time step in seconds
         Returns the control output (force) to apply.
        """
        error = target_z - current_z
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


if __name__ == '__main__':
    try:
        bridge = DroneCmdBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass

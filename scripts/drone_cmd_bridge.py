#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import ApplyBodyWrench
import tf
import xml.etree.ElementTree as ET #to parse urdf for mass data
import math

class DroneCmdBridge:
    def __init__(self):
        """
            Initializes the DroneCmdBridge node, sets up PID controller for altitude control, and subscribes to necessary topics.
            - Initializes ROS node and sets up parameters for the target body, hover force, and PID controller altitude stabilization.
            - Subscribes to
                - "cmd_vel" for velocity commands
                - "imu" for orientation data
                - "altitude" for current altitude updates.
                - "abs_z_target" for absolute altitude target updates
                - "rpy_stable_wrench" for drone planar staibilization demands
            - Prepares to apply wrenches to the drone in Gazebo based on the received commands and sensor data.
        """
        rospy.init_node('drone_cmd_bridge')
        self.UPDATE_HZ = 30

        self.current_wrench = Wrench()
        
        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("altitude", Float64, self.altitude_callback)
        rospy.Subscriber("abs_z_target", Float64, self.abs_z_target_callback) # for receiving absolute altitude targets if desired  
        self.rpy_stable_sub = rospy.Subscriber("rpy_stable_wrench", Wrench, self.rpy_stable_callback)
        self.state_pub = rospy.Publisher("bridge/state", String, queue_size=10)

        self.ns = rospy.get_namespace().strip('/')
        self.target_body = f"{self.ns}::link_drone_body"

        gravity = self.get_gazebo_gravity()
        total_mass = self.get_total_mass()
        self.HOVER_FORCE = gravity * total_mass
        self.FORCE_SCALE = 5.0
        self.TORQUE_SCALE = 0.1

        # PID for altitude stabilization
        self.elev_PID = ElevPIDController(kp=4, ki=0.01, kd=2)
        self.desired_z = None # desired altitude in meters
        self.current_z = None # current altitude in meters, updated from Gazebo
        self.current_yaw = None
        self.stabilization_torque = [0,0,0]
        self.desired_abs_z = -0.1 # if set to a positive value, this will override desired_z and the drone will try to maintain this absolute altitude instead of adjusting based on cmd_vel vertical component
        self.Z_WRENCH_SCALE = 1.7 

        self.DUR_BUFFER = rospy.Duration(1.5 / self.UPDATE_HZ) # force applied for update period

        self.current_wrench.force.z = self.HOVER_FORCE
        self.current_vel_x = 0
        self.current_vel_y = 0
        self.current_angvel_z = 0
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        rospy.sleep(0.5)

    def rpy_stable_callback(self, msg):
        """
            Updates self.stabilization_torque to torques provided by rpy stabilizer
        """
        self.stabilization_torque[0] = msg.torque.x
        self.stabilization_torque[1] = msg.torque.y
        self.stabilization_torque[2] = msg.torque.z
    
    def abs_z_target_callback(self, msg):
        """
            Updates the desired absolute altitude target for the drone.
        """
        self.desired_abs_z = msg.data
        self.desired_z = self.desired_abs_z
        return
    
    def vel_callback(self, msg):
        """
            Updates the current wrench based on the incoming cmd_vel message.
            - msg: geometry_msgs/Twist message containing desired linear and angular velocities.
            - The linear x and y components of the cmd_vel are scaled and rotated to world frame and set as forces in the current wrench
            - The linear z component of the cmd_vel is used to adjust the desired altitude (self.desired_z)
        """
        # if we do not have an absolute elevation target, we can use the vertical component of cmd_vel to adjust our desired altitude
        self.current_vel_x = msg.linear.x
        self.current_vel_y = msg.linear.y
        self.current_angvel_z = msg.angular.z
        
        if self.desired_abs_z < 0:
            if self.desired_z is None:
                self.desired_z = 0   
            self.desired_z += msg.linear.z  # Update desired altitude
        else:
            self.desired_z = self.desired_abs_z # if we have an absolute elevation target, ignore vertical component of cmd_vel and just use the absolute target
        
        self.desired_z = max(0.1, self.desired_z)  # Prevent going below ground level
        return

    def imu_callback(self, msg):
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)
        return

    def altitude_callback(self, msg):
        """
            Updates the current altitude of the drone (self.current_z)
        """
        self.current_z = msg.data
        return

    def run(self):
        """
            Main loop to continuously apply the current_wrench to the drone in Gazebo.
            - Uses a ROS rate to control the update frequency.
            - Calls update_wrench() to get the latest wrench demands from all sources
            - Applies the wrench using the /gazebo/apply_body_wrench service, specifying the target body and reference frame.
            - Catches any service exceptions and logs them.
        """
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown() and self.comms_not_established():
            self.state_pub.publish("Command Bridge Waiting on owner and depth cam")
            rate.sleep()

        while not rospy.is_shutdown():
            # update vertical force based on altitude error using PID
            self.update_wrench()
            rospy.wait_for_service('/gazebo/apply_body_wrench')
            try:
                self.apply_wrench(body_name=self.target_body,
                                  reference_frame="", # so that force applied in world frame
                                  wrench=self.current_wrench,
                                  duration=self.DUR_BUFFER)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

            state = self.make_state_msg()
            self.state_pub.publish(state)
            rate.sleep()
    
    def comms_not_established(self):
        """
            Checks if communications bridge needs are publishing messages.
            - desired_z (owner of cmd bridge or teleop twist)
            - current_z (ensure altitude publisher is active)
            - current_yaw (ensure imu is active)
            Returns True if any of the above are inactive (we are still waiting on first message)
        """
        if self.desired_z is None:
            return True
        if self.current_z is None:
            return True
        if self.current_yaw is None:
            return True
        return False
    
    def update_wrench(self):
        """
            Updates the current wrench force to be applied based on latest sensor and owner velocity
            requests and based on drone's current yaw. Rotates velocities from drone's frame into world's
            frame for call to gazebo apply_body_wrench service.
            - modifies self.current_wrench
        """
        self.update_current_wrench_z()
        # get local variables to avoid mid-math updates from callbacks
        fx_local = self.current_vel_x * self.FORCE_SCALE
        fy_local = self.current_vel_y * self.FORCE_SCALE
        tz_local = self.current_angvel_z * self.TORQUE_SCALE
        yaw = self.current_yaw

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)        
    
        fx_world = fx_local * cos_y - fy_local * sin_y
        fy_world = fx_local * sin_y + fy_local * cos_y

        self.current_wrench.force.x = min(max(fx_world, -50), 50)
        self.current_wrench.force.y = min(max(fy_world, -50), 50)
        tx_body = self.stabilization_torque[0]
        ty_body = self.stabilization_torque[1]

        self.current_wrench.torque.x = tx_body * cos_y - ty_body * sin_y
        self.current_wrench.torque.y = tx_body * sin_y + ty_body * cos_y
        self.current_wrench.torque.z = min(max(tz_local, -5), 5)

        return

    def update_current_wrench_z(self):
        """
            Update the vertical force component of the current wrench based on the altitude error using the elevation PID controller.
            Modifies self.current_wrench.force.z to be the hover force plus the PID output needed to correct altitude errors.
        """
        loc_desired_z = self.desired_z
        if loc_desired_z is None or self.desired_abs_z < 0:
            self.current_wrench.force.z = 0
        else:
            self.z_needed = self.Z_WRENCH_SCALE * self.elev_PID.update(loc_desired_z, self.current_z, 1.0 / self.UPDATE_HZ)
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

    def make_state_msg(self):
        # Calculate the components of the vertical force for clarity
        total_z_force = self.current_wrench.force.z
        
        # Formatting string with fixed columns
        msg = (
            f"{'Force Z:':<10} {total_z_force:>8.2f} N\n"
            f"{'Force X:':<10} {self.current_wrench.force.x:>8.2f} N\n"
            f"{'Force Y:':<10} {self.current_wrench.force.y:>8.2f} N\n"
        )
        return msg

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
        if self.previous_error == 0:
            self.previous_error = error
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

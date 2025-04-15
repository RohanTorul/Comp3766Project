#!/usr/bin/env python3
import rospy
import math
import numpy as np
from core import IKinBody,IKinSpace
from sensor_msgs.msg import JointState


class ROS_interface:
    def __init__(self):
        """
        ROS interface for UAV navigation and gimbal control
        Handles elevation, distance, and bearing data with gimbal orientation
        """
        rospy.init_node('uav_gimbal_controller')
        
        # Navigation metrics
        self.elevation = 0.0      # Angle in degrees
        self.distance = 0.0       # Distance in meters
        self.bearing = 0.0        # Bearing between where the uav is pointing and the target in degrees (-180 to 180)
        self.bank_angle = 0.0     # Bank angle in degrees
        self.pitch_angle = 0.0    # Pitch angle in degrees
        self.heading = 0.0       # where the UAV is pointing in degrees
        
        # Gimbal angles
        self.current_theta0 = 0.0  # Rotation around z-axis (yaw)
        self.current_theta1 = 0.0  # Rotation around y-axis (pitch)
        self.current_theta2 = 0.0  # Rotation around x-axis (roll)
        
        # ROS Publishers/Subscribers
        self.gimbal_pub = rospy.Publisher('/gimbal_joint_states', JointState, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # PID parameters
        self.Kp = 1  # Proportional gain
        self.Ki = 0.00 # Integral gain

        self.Kd = 0.3  # Derivative gain
        self.integral_error = [0.0, 0.0, 0.0]
        self.last_error = [0.0, 0.0, 0.0]
        self.set_gimble_theta(0, 0, 0)  # Initialize gimbal angles to zero
        
    def update_metrics(self,heading, elevation, distance, bearing, bank_angle, pitch_angle):
        """
        Update the navigation metrics
        :param elevation: Angle in degrees (positive = above horizon)
        :param distance: Distance in meters
        :param bearing: Bearing in degrees (-180 to 180, 0 = straight ahead)
        :param bank_angle: Bank angle in degrees
        :param pitch_angle: Pitch angle in degrees
        """
        #just for UAV orientation
        self.heading = heading

        self.elevation = elevation
        self.distance = distance
        self.bearing = bearing
        self.bank_angle = bank_angle
        self.pitch_angle = pitch_angle
        
    def joint_states_callback(self, msg):
        """
        Callback for current joint states (gimbal angles)
        """
        try:
            #extracting uav heading from joint base_link_to_uav_bearing
            self.heading = msg.position[msg.name.index('base_link_to_uav_bearing')]
            # Extract gimbal angles from joint states using joint names
            self.current_theta0 = msg.position[msg.name.index('uav_pitching_to_cylinder0')]
            self.current_theta1 = msg.position[msg.name.index('cylinder0_to_cylinder1')]
            self.current_theta2 = msg.position[msg.name.index('cylinder1_to_cylinder2')]
        except ValueError:
            rospy.logwarn("Gimbal joint names not found in joint_states")
            
    def get_current_gimble_theta(self):
        """
        Get the current gimbal angles (theta0, theta1, theta2)
        :return: Tuple of angles in radians (theta0, theta1, theta2)
        """
        return (self.current_theta0, self.current_theta1, self.current_theta2)
    
    def set_gimble_theta(self, target_theta0, target_theta1, target_theta2):
        """
        Set the gimbal angles (theta0, theta1, theta2)
        :param theta0: Angle in radians for gimbal 0 (z-axis)
        :param theta1: Angle in radians for gimbal 1 (y-axis)
        :param theta2: Angle in radians for gimbal 2 (x-axis)
        """
        gimbal_cmd = JointState()
        gimbal_cmd.name = ['base_link_to_uav_bearing','uav_bearing_to_uav_banking','uav_banking_to_uav_pitching','uav_pitching_to_cylinder0', 'cylinder0_to_cylinder1', 'cylinder1_to_cylinder2']
        gimbal_cmd.position = [math.radians(self.heading),math.radians(self.bank_angle),math.radians(self.pitch_angle),target_theta0, target_theta1, target_theta2]
        self.gimbal_pub.publish(gimbal_cmd)
        
    # def calculate_current_orientation(self):
    #     """
    #     Calculate the current end effector orientation in ground frame
    #     :return: Rotation matrix (Rx, Ry, Rz) in ground frame
    #     """
    #     # Convert gimbal angles to rotation matrix
    #     # Note: Order of rotations matters (typically Z-Y-X)
    #     Rz = np.array([
    #         [math.cos(self.current_theta0), -math.sin(self.current_theta0), 0],
    #         [math.sin(self.current_theta0), math.cos(self.current_theta0), 0],
    #         [0, 0, 1]
    #     ])
        
    #     Ry = np.array([
    #         [math.cos(self.current_theta1), 0, math.sin(self.current_theta1)],
    #         [0, 1, 0],
    #         [-math.sin(self.current_theta1), 0, math.cos(self.current_theta1)]
    #     ])
        
    #     Rx = np.array([
    #         [1, 0, 0],
    #         [0, math.cos(self.current_theta2), -math.sin(self.current_theta2)],
    #         [0, math.sin(self.current_theta2), math.cos(self.current_theta2)]
    #     ])
        
    #     # Combined rotation (Z-Y-X order)
    #     R = Rz @ Ry @ Rx
        
        # # Account for UAV orientation (bank and pitch)
        # bank_rad = math.radians(self.bank_angle)
        # pitch_rad = math.radians(self.pitch_angle)
        
        # R_bank = np.array([
        #     [1, 0, 0],
        #     [0, math.cos(bank_rad), -math.sin(bank_rad)],
        #     [0, math.sin(bank_rad), math.cos(bank_rad)]
        # ])
        
        # R_pitch = np.array([
        #     [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
        #     [0, 1, 0],
        #     [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        # ])
        
        # # Final orientation in ground frame
        # R_ground = R_pitch @ R_bank @ R
        
        # return R_ground
    def rotation_matrix_x(self,angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0],
                        [0, c, -s],
                        [0, s, c]])

    def rotation_matrix_y(self,angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s],
                        [0, 1, 0],
                        [-s, 0, c]])

    def rotation_matrix_z(self,angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0],
                        [s, c, 0],
                        [0, 0, 1]])
        
    def forward_kinematics(self,uav_bank,uav_pitch, uav_heading, gimbal_theta0, gimbal_theta1, gimbal_theta2):

        #Start with the UAV Transformation matrix:
        R_uav = self.rotation_matrix_z(uav_heading) @ self.rotation_matrix_y(uav_pitch) @ self.rotation_matrix_x(uav_bank)
        P_uav = np.zeros(3)
        T_uav = np.eye(4)
        T_uav[:3, :3] = R_uav
        T_uav[:3, 3] = P_uav

        #Then the gimbal transformation matrix:
        R_gimbal = self.rotation_matrix_z(gimbal_theta0) @ self.rotation_matrix_y(gimbal_theta1) @ self.rotation_matrix_x(gimbal_theta2)
        P_gimbal = np.zeros(3) 
        T_gimbal = np.eye(4)
        T_gimbal[:3, :3] = R_gimbal
        T_gimbal[:3, 3] = P_gimbal
        #Then the end effector transformation matrix:
        T_end_effector = T_gimbal @ T_uav
        R_end_effector = T_end_effector[:3, :3]

        return R_end_effector @ np.array([0, 0, 1])
    
    
    def compute_numerical_jacobian(self, uav_bank, uav_pitch, uav_heading, theta0, theta1, theta2, delta=1e-4):
        angles = np.array([theta0, theta1, theta2])
        J = np.zeros((3, 3))
        p0 = self.forward_kinematics(uav_bank, uav_pitch, uav_heading, *angles)

        for i in range(3):
            # Central difference for better accuracy
            angles_perturbed_plus = angles.copy()
            angles_perturbed_plus[i] += delta
            p_plus = self.forward_kinematics(uav_bank, uav_pitch, uav_heading, *angles_perturbed_plus)

            angles_perturbed_minus = angles.copy()
            angles_perturbed_minus[i] -= delta
            p_minus = self.forward_kinematics(uav_bank, uav_pitch, uav_heading, *angles_perturbed_minus)

            J[:, i] = (p_plus - p_minus) / (2 * delta)  # Central difference
        return J


#works,uses inverse Jacobian,  however tweaking the parameters is time consuming. Rather use IkinSpace/Ikinbody from core.py
    def numerical_ik(self,bank,pitch,heading, target_direction, initial_guess, max_iters=10, tol=1e-3, damping=0.1):
        theta = np.array(initial_guess)
        

        for _ in range(max_iters):
            current_dir = self.forward_kinematics(bank, pitch, heading, *theta)
            lambda_factor = 0.01
            error = target_direction - current_dir
            error_norm = np.linalg.norm(error)
            if error_norm < tol:
                break
            
            J = self.compute_numerical_jacobian(bank, pitch, heading, *theta)          
            d_theta = np.linalg.pinv(J) @ error_norm
            #d_theta = np.linalg.pinv(J) @ error
            theta += d_theta
            
            # # Clamp angles to avoid instability
            # theta[0] = max(min(theta[0],2*np.pi),-2*np.pi)  # Wrap yaw to [-π, π]
            theta[1] = np.clip(theta[1], -np.pi/2, np.pi/2)       # Limit pitch
            # theta[2] = np.clip(theta[2], -np.pi/4, np.pi/4)       # Limit roll
        
        return theta

    # def calculate_target_orientation(self):
    #     pitch = math.radians(self.pitch_angle)
    #     bank = math.radians(self.bank_angle)
    #     heading = math.radians(self.heading)
    #     elevation = math.radians(self.elevation)
    #     bearing = math.radians(self.bearing)
    #     cos_elevation = math.cos(elevation)
    #     sin_elevation = math.sin(elevation)
    #     cos_bearing = math.cos(bearing)
    #     sin_bearing = math.sin(bearing)
    #     c0 = math.cos(heading)
    #     s0 = math.sin(heading)
    #     c1 =math.cos(pitch)
    #     s1 = math.sin(pitch)
    #     c2 = math.cos(bank)
    #     s2 = math.sin(bank)
    #     target_theta0 = 0
    #     target_theta1 = 0
    #     target_theta2 = 0
    #     c3 = lambda : math.cos(target_theta0)
    #     s3 = lambda : math.sin(target_theta0)
    #     c4 = lambda : math.cos(target_theta1)
    #     s4 = lambda : math.sin(target_theta1)
    #     c5 = lambda : math.cos(target_theta2)
    #     s5 = lambda : math.sin(target_theta2)


    #     Blist = np.array([
    #         [0,  0,  1,  0,  0,  0],
    #         [0,  1,  0, -1,  0,  0],
    #         [1,  0,  0,  0,  1,  0]
    #     ],dtype=float).T
    #     r03 = np.array([[c0*c1, c0*s1*s2-s0*c2,c0*s1*c2+s0*s2],
    #                       [s0*c1, s0*s1*s2+c0*c2,s0*s1*c2-c0*s2],
    #                       [-s1,   c1*s2,          c1*c2]])
    #     inv_r03 = np.linalg.inv(r03)

    #     r_gimbal = lambda:  np.array([[c3()*c4(), c3()*s4()*s5()-s3()*c5(),c3()*s4()*c5()+s3()*s5()],
    #                          [s3()*c4(), s3()*s4()*s5()+c3()*c5(),s3()*s4()*c5()-c3()*s5()],
    #                          [-s4(),   c4()*s5(),          c4()*c5()]])
        
        

    #     r_06 = np.array([[cos_elevation*cos_bearing, -sin_bearing, cos_bearing*sin_elevation],
    #                                       [sin_bearing*cos_elevation, cos_bearing, sin_bearing*sin_elevation],
    #                                       [-sin_elevation,          0,           cos_elevation]])
    #     r36 = np.dot(inv_r03,r_06)

    #     # Solve for target_theta1 (originally θ4)
    #     target_theta1 = -np.arcsin(r36[2, 0])  # From R13 = -sin(θ4)
    #     target_theta0 = np.arcsin(r36[1,0]/np.cos(target_theta1))
    #     target_theta2 = np.arcsin(r36[2,1]/np.cos(target_theta1))  # From R11 = cos(θ4)*cos(θ3)
        
    #     # if not np.isclose(np.cos(target_theta1), 0):  # General case (no gimbal lock)
    #     #     # Solve for target_theta0 (originally θ3)
    #     #     target_theta0 = np.arctan2(r36[1, 0], r36[0, 0])  # From R11/R12
            
    #     #     # Solve for target_theta2 (originally θ5)
    #     #     target_theta2 = np.arctan2(r36[1, 2], r36[2, 2])  # From R23/R33
    #     # else:  # Gimbal lock case (cos(θ4) = 0)
    #     #     target_theta0 = 0  # Arbitrary choice when singular
    #     #     if np.isclose(target_theta1, np.pi/2):  # θ4 = +90°
    #     #         target_theta2 = np.arctan2(r36[0, 1], r36[1, 1])
    #     #     else:  # θ4 = -90°
    #     #         target_theta2 = np.arctan2(-r36[0, 1], -r36[1, 1])

    #     print(r36 == r_gimbal())
    #     return [target_theta0, target_theta1, target_theta2]

    def Find_thetas(self):
        
        # Convert angles to radians (preserve original elevation sign)
        pitch = -math.radians(self.pitch_angle)
        bank = math.radians(self.bank_angle)
        heading = math.radians(self.heading)
        elevation = -math.radians(self.elevation) #empirically determined
        bearing = math.radians(self.bearing)

        #"spherical" coodinates of target to cartesian
        target_dir = np.array([
            math.cos(elevation) * math.cos(bearing),
            math.cos(elevation) * math.sin(bearing),
            math.sin(elevation) 
        ])
        #Find out what the target direction is in the UAV body frame
        R = (self.rotation_matrix_z(heading) @ 
            self.rotation_matrix_y(pitch) @ 
            self.rotation_matrix_x(bank))

        # Transform target to uav frame, working backwards
        target_body = R.T @ target_dir  # Equivalent to inv(R) @ target_dir because rotation matrices are unitary

        # Calculate gimbal angles
        target_theta0 = math.atan2(target_body[1], target_body[0])  # Yaw
        
        # CORRECTED PITCH CALCULATION:
        xy_mag = math.hypot(target_body[0], target_body[1])
        target_theta1 = -math.atan2(target_body[2], xy_mag)  #empirically determined
        
        # Roll compensation (simplified)
        target_theta2 = -bank  # Basic compensation

        # Apply angle limits
        target_theta0 = (target_theta0 + math.pi) % (2*math.pi) - math.pi
        target_theta1 = np.clip(target_theta1, -math.pi/2, math.pi/2)  # Physical pitch limits
        target_theta2 = np.clip(target_theta2, -math.pi/4, math.pi/4)  # Physical roll limits

        return target_theta0, -target_theta1, target_theta2
    

    #PID is not ideal for this application. It has been tested and results have been better without it.(PID control made the gimbal move erratically), Due to time constraints, the implementation has been commented out.
        
    # def calculate_pid(self, target_angle, current_angle, axis):
    #     """
    #     Calculate PID control signal
    #     :param target_angle: Target angle in radians
    #     :param current_angle: Current angle in radians
    #     :param axis: Axis index (0=z, 1=y, 2=x)
    #     :return: PID control output in radians
    #     """
    #     error = target_angle - current_angle
        
    #     # Proportional term
    #     p = self.Kp * error
        
    #     # Integral term (with anti-windup)
    #     self.integral_error[axis] += error*0.01
    #     i = self.Ki * self.integral_error[axis]
        
    #     # Derivative term
    #     d = self.Kd * (error - self.last_error[axis])/0.01
    #     self.last_error[axis] = error
        
    #     return p + i + d
        
    # def set_gimble_thetas_pid(self, theta0, theta1, theta2):
    #     """
    #     Set gimbal angles using PID control
    #     :param theta0: Target angle in radians for gimbal 0 (z-axis)
    #     :param theta1: Target angle in radians for gimbal 1 (y-axis)
    #     :param theta2: Target angle in radians for gimbal 2 (x-axis)
    #     """
    #     current_theta0, current_theta1, current_theta2 = self.get_current_gimble_theta()
        
    #     pid_theta0 = current_theta0 + self.calculate_pid(theta0, current_theta0, 0)
    #     pid_theta1 = current_theta1 + self.calculate_pid(theta1, current_theta1, 1)
    #     pid_theta2 = current_theta2 + self.calculate_pid(theta2, current_theta2, 2)
        
    #     self.set_gimble_theta(pid_theta0, pid_theta1, pid_theta2/2)



    #called by uav_sim.py
    def auto_point_gimbal(self):
        target_thetas = self.Find_thetas()
        #target_thetas = self.calculate_target_orientation()
        self.set_gimble_theta(*target_thetas)
        #self.set_gimble_thetas_pid(*target_thetas)

if __name__ == "__main__":
    try:
        controller = ROS_interface()
        
        # Example usage
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # Simulate detecting target at 30° bearing, 15° elevation, 50m distance
            # With UAV at 10° bank and 5° pitch
            controller.update_metrics(
                elevation=15.0,
                distance=50.0,
                bearing=30.0,
                bank_angle=10.0,
                pitch_angle=5.0
            )
            
            # Auto-point gimbal
            controller.auto_point_gimbal()
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
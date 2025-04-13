#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class GazeboRobotControl:
    def __init__(self):
        rospy.init_node('gazebo_robot_control')
        
        # Load parameters
        self.load_parameters()
        
        # Initialize action client
        self.setup_action_client()
        
        # Initialize trajectory generation parameters
        self.current_positions = np.zeros(len(self.joint_names))
        self.target_positions = np.zeros(len(self.joint_names))
        self.last_update_time = rospy.Time.now()
        
        # Setup subscriber with buffering
        rospy.Subscriber(
            'robot_config', 
            Float32MultiArray, 
            self.config_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("Gazebo robot control initialized")

    def load_parameters(self):
        """Load and validate configuration parameters"""
        self.joint_names = rospy.get_param(
            '~joint_names',
            ['base_link_to_cylinder1', 'cylinder1_to_cylinder2', 'cylinder2_to_cylinder3']
        )
        
        self.joint_limits = rospy.get_param(
            '~joint_limits',
            [
                [-2.0, 2.0],    # base_link_to_cylinder1
                [-1.55, 0.0],   # cylinder1_to_cylinder2 
                [-2.0, 2.0]     # cylinder2_to_cylinder3
            ]
        )
        
        self.trajectory_params = rospy.get_param(
            '~trajectory',
            {
                'min_duration': 0.1,  # Minimum movement time (seconds)
                'max_duration': 0.2,   # Maximum movement time
                'velocity_scale': 0.9, # Fraction of max velocity to use
                'accel_scale': 0.9     # Fraction of max acceleration
            }
        )
        
        # Validate parameters
        if len(self.joint_names) != len(self.joint_limits):
            rospy.logerr("Joint names and limits mismatch!")
            rospy.signal_shutdown("Configuration error")

    def setup_action_client(self):
        """Initialize and wait for action server"""
        self.client = actionlib.SimpleActionClient(
            '/joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("Timed out waiting for action server!")
            rospy.signal_shutdown("Action server not available")
            return
        
        rospy.loginfo("Connected to trajectory action server")

    def config_callback(self, msg):
        """Handle incoming configuration messages"""
        try:
            if len(msg.data) == len(self.joint_names):
                self.process_new_target(msg.data)
            else:
                rospy.logwarn_once(
                    f"Received {len(msg.data)} values, expected {len(self.joint_names)}"
                )
        except Exception as e:
            rospy.logerr(f"Error processing message: {str(e)}")

    def process_new_target(self, target_positions):
        """Process and validate new target positions"""
        # Apply joint limits
        clamped_positions = [
            max(min_val, min(max_val, pos))
            for pos, (min_val, max_val) in zip(target_positions, self.joint_limits)
        ]
        
        # Update target
        self.target_positions = np.array(clamped_positions)
        self.last_update_time = rospy.Time.now()
        
        # Calculate trajectory duration based on movement distance
        position_deltas = np.abs(self.target_positions - self.current_positions)
        max_delta = np.max(position_deltas)
        
        # Scale duration based on movement distance
        duration = min(
            max(
                self.trajectory_params['min_duration'],
                max_delta * self.trajectory_params['velocity_scale']
            ),
            self.trajectory_params['max_duration']
        )
        
        self.send_trajectory(duration)

    def send_trajectory(self, duration):
        """Send smooth trajectory to Gazebo controller"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # Create trajectory points
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_positions.tolist()
        start_point.time_from_start = rospy.Duration(0)
        
        end_point = JointTrajectoryPoint()
        end_point.positions = self.target_positions.tolist()
        end_point.time_from_start = rospy.Duration(duration)
        
        # Optionally add intermediate points for smoother motion
        goal.trajectory.points = [start_point, end_point]
        
        # Send goal with timeout
        self.client.send_goal(goal)
        self.current_positions = self.target_positions.copy()
        
        rospy.loginfo_throttle(
            1.0,
            f"Moving to: {[round(p, 4) for p in self.target_positions]} "
            f"in {duration:.2f}s"
        )

    def run(self):
        """Main loop for any periodic tasks"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Add any periodic checks here
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = GazeboRobotControl()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Gazebo control node")
    except Exception as e:
        rospy.logerr(f"Fatal error in Gazebo control: {str(e)}")
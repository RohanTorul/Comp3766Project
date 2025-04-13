#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class RobotControl:
    def __init__(self):
        rospy.init_node('robot_control')
        
        # Load parameters with defaults
        self.load_parameters()
        
        # Initialize joint state
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state.position = [0.0] * len(self.joint_names)
        
        # Setup publisher with latch=False for real-time control
        self.joint_pub = rospy.Publisher(
            '/joint_states', 
            JointState, 
            queue_size=10,
            tcp_nodelay=True  # Reduce latency
        )
        
        # Subscriber with buffered messages
        rospy.Subscriber(
            'robot_config', 
            Float32MultiArray, 
            self.config_callback,
            queue_size=1,  # Only process latest message
            buff_size=2**24  # Larger buffer for high rates
        )
        
        # Diagnostics
        self.last_update_time = rospy.Time.now()
        self.update_count = 0
        self.control_rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"Robot control node ready. Controlling joints: {self.joint_names}")

    def load_parameters(self):
        """Load and validate configuration parameters"""
        self.joint_names = rospy.get_param(
            '~joint_names',
            ['base_link_to_cylinder1', 'cylinder1_to_cylinder2', 'cylinder2_to_cylinder3']
        )
        
        # Joint limits: [min, max] for each joint
        self.joint_limits = rospy.get_param(
            '~joint_limits',
            [
                [-1.0, 1.0],    # base_link_to_cylinder1
                [-1.55, 0.0],   # cylinder1_to_cylinder2 
                [-1.0, 1.0]     # cylinder2_to_cylinder3
            ]
        )
        
        # Rate parameters
        self.publish_rate = rospy.get_param('~publish_rate', 30)  # Hz
        self.status_rate = rospy.get_param('~status_rate', 1)     # Status messages/sec
        
        # Validate parameters
        if len(self.joint_names) != 3 or len(self.joint_limits) != 3:
            rospy.logerr("Invalid joint configuration! Must specify 3 joints.")
            rospy.signal_shutdown("Invalid configuration")
            return

    def config_callback(self, msg):
        """Handle incoming configuration messages"""
        try:
            if len(msg.data) == len(self.joint_names):
                self.update_joints(msg.data)
            else:
                rospy.logwarn_once(
                    f"Received {len(msg.data)} values, expected {len(self.joint_names)}"
                )
        except Exception as e:
            rospy.logerr(f"Error processing message: {str(e)}")

    def update_joints(self, target_positions):
        """Update joint positions with safety checks"""
        updated = False
        for i, (pos, (min_limit, max_limit)) in enumerate(zip(target_positions, self.joint_limits)):
            # Apply smooth filtering if needed (commented out by default)
            # filtered_pos = 0.9 * self.joint_state.position[i] + 0.1 * pos
            
            # Apply hard limits
            clamped_pos = max(min_limit, min(max_limit, pos))
            
            if abs(self.joint_state.position[i] - clamped_pos) > 0.001:  # Deadband
                self.joint_state.position[i] = clamped_pos
                updated = True
        
        if updated:
            self.joint_state.header.stamp = rospy.Time.now()
            self.last_update_time = rospy.Time.now()
            self.update_count += 1
            
            # Throttled status logging
            if self.update_count % (self.publish_rate // self.status_rate) == 0:
                rospy.loginfo_throttle(
                    1.0,
                    f"Joints updated to: {[round(p, 4) for p in self.joint_state.position]}"
                )

    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting control loop...")
        
        while not rospy.is_shutdown():
            # Continuous publishing maintains smooth visualization
            self.joint_pub.publish(self.joint_state)
            
            # Safety check - warn if no updates received
            if (rospy.Time.now() - self.last_update_time).to_sec() > 5.0:
                rospy.logwarn_throttle(5.0, "No joint updates received for 5 seconds")
            
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotControl()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down robot control node")
    except Exception as e:
        rospy.logerr(f"Fatal error: {str(e)}")
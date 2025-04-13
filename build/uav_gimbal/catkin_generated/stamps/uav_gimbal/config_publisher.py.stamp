#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray

class ConfigPublisher:
    def __init__(self):
        rospy.init_node('config_publisher', anonymous=True)
        
        # Parameters with default values (can be modified via ROS params)
        self.publish_rate = rospy.get_param('~publish_rate', 100)  # Hz
        self.frequency = rospy.get_param('~frequency', 1.0)  # Hz
        self.amplitudes = rospy.get_param('~amplitudes', [1.0, 0.8, 1.0])
        self.offsets = rospy.get_param('~offsets', [0.0, -0.8, 0.0])
        self.phase_multipliers = rospy.get_param('~phase_multipliers', [1.0, 0.5, 1.0])
        self.scale_factor = rospy.get_param('~scale_factor', 10.0)
        
        self.pub = rospy.Publisher('robot_config', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(self.publish_rate)
        self.time_var = 0.0
        self.log_counter = 0
        
    def generate_trajectory(self):
        """Generate smooth circular trajectory for all joints"""
        angles = [
            self.scale_factor * (self.offsets[i] + 
            self.amplitudes[i] * math.sin(self.phase_multipliers[i] * self.frequency * self.time_var))
            for i in range(3)
        ]
        return angles
    
    def run(self):
        while not rospy.is_shutdown():
            angles = self.generate_trajectory()
            
            msg = Float32MultiArray()
            msg.data = angles
            self.pub.publish(msg)
            
            # Log at about 1Hz regardless of publish rate
            if self.log_counter % self.publish_rate == 0:
                rospy.loginfo(f"Published angles: {[round(a, 3) for a in angles]}")
            self.log_counter += 1
            
            self.time_var += 1.0/self.publish_rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = ConfigPublisher()
        rospy.loginfo("Starting circular trajectory publisher...")
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down trajectory publisher")
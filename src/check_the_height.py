#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class HeightCondition:
    def __init__(self, height_threshold):
        """
        Initialize the HeightCondition class.

        Parameters:
        height_threshold (float): The height threshold in meters.
        """
        # Initialize the ROS node
        rospy.init_node('height_condition_node', anonymous=False)
        
        # Define the height threshold
        self.height_threshold = height_threshold
        
        # Subscribe to the appropriate topic
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        # Uncomment the following line and comment the above line to use the global position topic
        # self.global_position_sub = rospy.Subscriber('/mavros/global_position/local', NavSatFix, self.global_position_callback)
        
        # Initialize the current height
        self.current_height = None
        
        # Publisher for the condition result
        self.condition_pub = rospy.Publisher('/check_the_height_success', Bool, queue_size=1)

    def pose_callback(self, msg):
        """
        Callback function for local position pose.

        Parameters:
        msg (PoseStamped): The PoseStamped message containing position data.
        """
        self.current_height = msg.pose.position.z
        self.check_height()

    def check_height(self):
        """
        Check if the current height meets the required threshold.
        """
        if self.current_height is not None:
            # Print current height
            rospy.loginfo("Current height: %.2f meters", self.current_height)
            
            if self.current_height >= self.height_threshold:
                result = True
                rospy.loginfo("Height condition met: %.2f meters", self.current_height)
            else:
                result = False
                rospy.logwarn("Height condition not met: %.2f meters", self.current_height)
            self.condition_pub.publish(result)
        else:
            rospy.logwarn("Current height data not available")

if __name__ == '__main__':
    height_threshold = rospy.get_param('~height_threshold', 10.0)  # Default threshold is 10 meters
    HeightCondition(height_threshold)
    rospy.spin()

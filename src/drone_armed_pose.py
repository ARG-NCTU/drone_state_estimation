#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class DroneArmedPose:
    """
    A ROS node that subscribes to the drone's pose and state information and publishes the pose 
    when the drone is armed. This node monitors both the Gazebo simulation pose and the MAVROS 
    pose, recording the pose at the time the drone is armed and continuously publishing it while 
    the drone remains armed.

    Subscribers:
        - /gazebo/drone/pose (PoseStamped): Current pose of the drone in the Gazebo simulation.
        - /mavros/local_position/pose (PoseStamped): Current pose of the drone as provided by MAVROS.
        - /mavros/state (State): Current state of the drone, including its armed status.

    Publishers:
        - /drone/armed_pose (PoseStamped): Pose of the drone when it was armed (Gazebo simulation).
        - /drone/mavros/armed_pose (PoseStamped): Pose of the drone when it was armed (MAVROS).
    """
    def __init__(self):
        # Subscribers
        self.pose_sub = rospy.Subscriber('/gazebo/drone/pose', PoseStamped, self.pose_callback)
        self.pose_mavros_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

        # Publisher
        self.pose_pub = rospy.Publisher('/drone/armed_pose', PoseStamped, queue_size=10)
        self.mavros_pose_pub = rospy.Publisher('/drone/mavros/armed_pose', PoseStamped, queue_size=10)

        self.pose_timer = rospy.Timer(rospy.Duration(0.1), self.pose_check)

        self.drone_armed = False
        self.initial_pose_recorded = False
        self.gazebo_current_pose = None
        self.mavros_current_pose = None
        self.gazebo_arm_pose = None
        self.mavros_arm_pose = None

    def pose_callback(self, data):
        self.gazebo_current_pose = data

    def mavros_pose_callback(self, data):
        self.mavros_current_pose = data

    def state_callback(self, data):
        self.drone_armed = data.armed
        if self.drone_armed:
            rospy.loginfo("Drone is armed")
        else:
            rospy.loginfo("Drone is disarmed")
            self.initial_pose_recorded = False

    
    def pose_check(self, event):
        if self.drone_armed and not self.initial_pose_recorded:
            rospy.logwarn("Drone is armed and record intitial pose")
            self.gazebo_arm_pose = self.gazebo_current_pose
            self.mavros_arm_pose = self.mavros_current_pose
            self.initial_pose_recorded = True
        elif self.drone_armed and self.initial_pose_recorded:
            self.pose_pub.publish(self.gazebo_arm_pose)
            self.mavros_pose_pub.publish(self.mavros_arm_pose)
            rospy.loginfo("Publishing armed pose")
        elif not self.drone_armed or self.initial_pose_recorded:
            pass
            

if __name__ == '__main__':
    try:
        rospy.init_node('drone_armed_pose', anonymous=True)
        DroneArmedPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
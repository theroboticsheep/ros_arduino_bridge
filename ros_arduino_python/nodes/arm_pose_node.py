#!/usr/bin/env python


import rospy
from ros_arduino_msgs.srv import *
import os, time
import thread

class ArmPose():
    def __init__(self):
        rospy.init_node('arm_pose', log_level=rospy.DEBUG)
        
        self.animation = rospy.get_param("animation", "~proto-demo")
        self.sequence = rospy.get_param("sequence", "~sequence1")
        self.pose_name = rospy.get_param("pose_name", "~home")
        rospy.loginfo("animation name: " + self.animation)
        rospy.loginfo("sequence name: " + self.sequence)
        rospy.loginfo("pose name: " + self.pose_name)
        
        self.poses = rospy.get_param(self.animation, dict({}))
        
        rospy.wait_for_service('/arduino/servo_write')
        servo_write = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
        
        for pose in self.poses[self.sequence]:
            rospy.loginfo("Pose set: " + pose)
            for joint, params in self.poses[pose].iteritems():
                servo_write(params['joint_num'],params['joint_angle'])
            time.sleep(7)

        rospy.loginfo("Command sent to servo")
        rospy.signal_shutdown("Shutting down arm_pose node")

if __name__ == '__main__':
    myArmPose = ArmPose()
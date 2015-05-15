#!/usr/bin/env python


import rospy
from ros_arduino_msgs.srv import *
import os, time
import thread

class ArmPose():
    def __init__(self):
        rospy.init_node('arm_pose', log_level=rospy.DEBUG)

        self.joint_num = rospy.get_param("joint_num", 0)
        self.joint_angle = rospy.get_param("joint_angle", 0)
        
        rospy.loginfo("Received joint number " + str(self.joint_num) + " and angle: " + str(self.joint_angle) + " radians")
        
        rospy.wait_for_service('/arduino/servo_write')
        servo_write = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
        response = servo_write(self.joint_num, self.joint_angle)

        rospy.loginfo("Command sent to servo")
        rospy.signal_shutdown("Shutting down arm_pose node")

if __name__ == '__main__':
    myArmPose = ArmPose()
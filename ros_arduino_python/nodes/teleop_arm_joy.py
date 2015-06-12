#!/usr/bin/env python

import rospy, time
from ros_arduino_msgs.srv import *
from sensor_msgs.msg import Joy
from itertools import cycle


class TeleopArmJoy():
    def __init__(self):
        self.msg = """
        ***************************************************
        *                                                 *
        * Press 'A' button to advance to the next pose.   *
        * Pose sequences defined in config/arm_poses.yaml *
        *                                                 *
        * Parameters:                                     *
        *     animation - default:="~proto-demo"          *
        *                                                 *
        ***************************************************
        """
        
        rospy.init_node('teleop_arm_joy')
        self.animation = rospy.get_param("animation", "~proto-demo")
        self.poses = rospy.get_param(self.animation, dict({}))
         
        rospy.loginfo("Waiting for rosservice: servo_write")
        rospy.wait_for_service('/arduino/servo_write')
        self.servo_write = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
        rospy.loginfo("Connected to rosservice: servo_write")
        rospy.loginfo(self.msg)
        
        self.t_delta = rospy.Duration(.25)
        self.t_a = rospy.Time.now() + self.t_delta
        self.t_b = rospy.Time.now() + self.t_delta
        self.t_x = rospy.Time.now() + self.t_delta
        self.t_y = rospy.Time.now() + self.t_delta
        self.t_LB = rospy.Time.now() + self.t_delta
        self.t_RB = rospy.Time.now() + self.t_delta
        self.t_back  = rospy.Time.now() + self.t_delta
        self.t_start = rospy.Time.now() + self.t_delta
        self.t_power = rospy.Time.now() + self.t_delta
        self.t_left_stick  = rospy.Time.now() + self.t_delta
        self.t_right_stick = rospy.Time.now() + self.t_delta
        
        self.iter_a = cycle(self.poses["sequence_a"])
        self.iter_b = cycle(self.poses["sequence_b"])
        self.iter_x = cycle(self.poses["sequence_x"])
        self.iter_y = cycle(self.poses["sequence_y"])
        self.iter_LB = cycle(self.poses["sequence_LB"])
        self.iter_RB = cycle(self.poses["sequence_RB"])
        self.iter_back  = cycle(self.poses["sequence_back"])
        self.iter_start = cycle(self.poses["sequence_start"])
        self.iter_power = cycle(self.poses["sequence_power"])
        self.iter_left_stick  = cycle(self.poses["sequence_left_stick"])
        self.iter_right_stick = cycle(self.poses["sequence_right_stick"])
        
        rospy.Subscriber("joy", Joy, self.callback)
        
        rospy.spin()

    def doPose(self, pose):
        rospy.loginfo("Pose set: " + pose)
        for joint, params in self.poses[pose].iteritems():
            temp = 1
            self.servo_write(params['joint_num'],params['joint_angle'])

    def callback(self, data):
        now = rospy.Time.now()
        # prevent multiple button reads
        if now > self.t_a:
            if data.buttons[0]:
                self.doPose(next(self.iter_a))
                self.t_a = now + self.t_delta
        if now > self.t_b:
            if data.buttons[1]:
                self.doPose(next(self.iter_b))
                self.t_b = now + self.t_delta
        if now > self.t_x:
            if data.buttons[2]:
                #reserved for teleop_twist
                #self.doPose(next(self.iter_x))
                self.t_x = now + self.t_delta
        if now > self.t_y:
            if data.buttons[3]:
                self.doPose(next(self.iter_y))
                self.t_y = now + self.t_delta
        if now > self.t_LB:
            if data.buttons[4]:
                self.doPose(next(self.iter_LB))
                self.t_LB = now + self.t_delta
        if now > self.t_RB:
            if data.buttons[5]:
                #reserved for teleop_twist
                #self.doPose(next(self.iter_RB))
                self.t_RB = now + self.t_delta
        if now > self.t_back:
            if data.buttons[8]:
                self.doPose(next(self.iter_back))
                self.t_back = now + self.t_delta
        if now > self.t_start:
            if data.buttons[9]:
                self.doPose(next(self.iter_start))
                self.t_start = now + self.t_delta
        if now > self.t_power:
            if data.buttons[16]:
                self.doPose(next(self.iter_power))
                self.t_power = now + self.t_delta
        if now > self.t_left_stick:
            if data.buttons[10]:
                self.doPose(next(self.iter_left_stick))
                self.t_left_stick = now + self.t_delta
        if now > self.t_right_stick:
            if data.buttons[11]:
                self.doPose(next(self.iter_right_stick))
                self.t_right_stick = now + self.t_delta

if __name__ == '__main__':
    teleopArmJoy = TeleopArmJoy()

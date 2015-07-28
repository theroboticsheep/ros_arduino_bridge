#!/usr/bin/env python

import rospy, time
from ros_arduino_msgs.srv import *
from sensor_msgs.msg import Joy
from itertools import cycle


class TeleopArmJoy():
    def __init__(self):
        self.msg = """
        *****************************************************
        *                                                   *
        * Press buttons to advance through sequenced poses. *
        * Pose sequences defined in config/arm_poses.yaml   *
        *                                                   *
        * Parameters:                                       *
        *     animation - default:="~proto-demo"            *
        *                                                   *
        *****************************************************
        """
        
        rospy.init_node('teleop_demo')
        self.animation = rospy.get_param('animation', '~proto-demo')
        self.poses = rospy.get_param(self.animation, dict({}))
         
        rospy.loginfo('Waiting for rosservice: servo_write')
        rospy.wait_for_service('/arduino_bridge/servo_write')
        self.updroid_servo_write = rospy.ServiceProxy('/arduino_bridge/servo_write', ServoWrite)
        rospy.loginfo('Connected to rosservice: servo_write')
        rospy.loginfo(self.msg)
        
        buttons = [['a',0],['b',1],['y',3],['LB',4],['back',8],['start',9],['power',16],['left_stick',10],['right_stick',11]]
        self.button_list = dict()
        for button in buttons:
            self.button_list[button[0]] = Button(button[0],button[1], self.poses, self.updroid_servo_write)
        
        rospy.Subscriber('joy', Joy, self.callback)


        #Set button sequences
        self.button_list['a'].setSequence('sequence_a')




        rospy.spin()

    def callback(self, data):
        now = rospy.Time.now()
        
        for key, button in self.button_list.iteritems():
            if now > button.t_next:
                if data.buttons[button.button_id]:
                    button.doPose()
                    button.t_next = now + button.t_delta
        
class Button:
    def __init__(self, button_name, button_id, poses, updroid_servo_write):
        self.button_name = button_name
        self.button_id = button_id
        self.poses = poses
        self.updroid_servo_write = updroid_servo_write
        self.t_delta = rospy.Duration(.25)
        self.t_next = rospy.Time.now() + self.t_delta
        self.sequence = cycle(self.poses['sequence_a'])
        
    def setSequence(self, sequence):
        self.sequence = cycle(self.poses[sequence])
        
    def doPose(self):
        pose = next(self.sequence)
        rospy.loginfo('Pose set: ' + pose)
        for joint, params in self.poses[pose].iteritems():
            self.updroid_servo_write(params['joint_num'],params['joint_angle'])
        

if __name__ == '__main__':
    teleopArmJoy = TeleopArmJoy()

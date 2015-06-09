#!/usr/bin/env python

import rospy, time
from ros_arduino_msgs.srv import *
from sensor_msgs.msg import Joy


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
        *     sequence  - default:="sequence1"            *
        *                                                 *
        ***************************************************
        """
        
        rospy.init_node('teleop_arm_joy')
        
        now = rospy.Time.now()
        self.t_delta = rospy.Duration(.25)
        self.t_next = now + self.t_delta

        self.animation = rospy.get_param("animation", "~proto-demo")
        self.sequence = rospy.get_param("sequence", "~sequence1")
        rospy.loginfo("animation: " + self.animation)
        rospy.loginfo("sequence: " + self.sequence)
         
        self.poses = rospy.get_param(self.animation, dict({}))
         
        rospy.loginfo("Waiting for rosservice: servo_write")
        rospy.wait_for_service('/arduino/servo_write')
        self.servo_write = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
        rospy.loginfo("Connected to rosservice: servo_write")
        rospy.loginfo(self.msg)
        
        self.button_a_pressed = False
        rospy.Subscriber("joy", Joy, self.callback)

        done = False
        while 1:
            for pose in self.poses[self.sequence]:
                # wait until button is pressed
                while not self.button_a_pressed:
                    continue
                rospy.loginfo("Pose set: " + pose)
                self.button_a_pressed = False
                for joint, params in self.poses[pose].iteritems():
                    self.servo_write(params['joint_num'],params['joint_angle'])
            if done:
                break
     
        rospy.signal_shutdown("Shutting down teleop_arm_joy node")

#     Index   Button
#     0       A
#     1       B
#     2       X
#     3       Y
#     4       LB
#     5       RB
#     6       back
#     7       start
#     8       power
#     9       Button stick left
#     10      Button stick right
    def callback(self, data):
        button_a = 0
        now = rospy.Time.now()
        # prevent multiple button reads
        if now > self.t_next:
            if data.buttons[button_a]:
                self.button_a_pressed = True
                self.t_next = now + self.t_delta

if __name__ == '__main__':
    teleopArmJoy = TeleopArmJoy()

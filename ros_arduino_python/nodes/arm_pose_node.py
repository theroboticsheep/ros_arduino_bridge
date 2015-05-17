#!/usr/bin/env python

import rospy
from ros_arduino_msgs.srv import *
import os, time
import thread
import sys, select, termios, tty

msg = """
***************************************************
*                                                 *
* Press any key to advance to the next pose.      *
* Pose sequences defined in config/arm_poses.yaml *
*                                                 *
* Parameters:                                     *
*     animation - default:="~proto-demo"          *
*     sequence  - default:="sequence1"            *
*                                                 *
* CTRL-C to quit                                  *
*                                                 *
***************************************************
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('arm_pose')
    settings = termios.tcgetattr(sys.stdin)
    
    animation = rospy.get_param("animation", "~proto-demo")
    sequence = rospy.get_param("sequence", "~sequence1")
    rospy.loginfo("animation: " + animation)
    rospy.loginfo("sequence: " + sequence)
    
    poses = rospy.get_param(animation, dict({}))
    
    rospy.loginfo("Waiting for rosservice: servo_write")
    rospy.wait_for_service('/arduino/servo_write')
    servo_write = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
    rospy.loginfo("Connected to rosservice: servo_write")
    rospy.loginfo(msg)
    
    done = False
    while(1):
        for pose in poses[sequence]:
            key = getKey()
            # ctrl-c to quit
            if (key == '\x03'):
                done = True
                break
            rospy.loginfo("Pose set: " + pose)
            for joint, params in poses[pose].iteritems():
                servo_write(params['joint_num'],params['joint_angle'])
        if done:
            break

    rospy.signal_shutdown("Shutting down arm_pose node")
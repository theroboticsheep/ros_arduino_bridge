#!/usr/bin/env python

"""
    A joint controller class for the Arduino microcontroller
    Assumes standard hobby servos
    
    Borrowed heavily from Mike Feguson's ArbotiX code.
    
    Created for the UpDroid: http://www.updroid.com
    Author: Nathaniel Gallinger

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy, actionlib
import os

from std_msgs.msg import Float64
from math import pi, radians
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory

class HobbyServo:

    def __init__(self, controller, name):
        self.controller = controller
        self.name = name
        
        ns = controller.joint_dict+'/'+name+'/'
        
        self.id = int(rospy.get_param(ns+'id'))
        self.range = int(rospy.get_param(ns+'range', 180))
        
        msg = 'Joint added: ' + ns + ', id: ' + str(self.id) + ' range: ' + str(self.range)
        rospy.loginfo(msg)

        self.dirty = False                      # newly updated position?
        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.cmd_prev = 0.0                     # previous position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.time_prev = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def setCurrentFeedback(self, reading):
        """ Update angle in radians by reading from servo """
        # check validity
        if reading >= -(pi/2) and reading <= (pi/2):
            angle_prev = self.position
            self.position = reading
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - angle_prev)/((t - self.time_prev).to_nsec()/1000000000.0)
            self.time_prev = t
        else:
            msg = 'Invalid read of servo: id ' + str(self.id) + ', value ' + str(reading)
            rospy.logdebug(msg)
            return
        if not self.controller.active:
            self.cmd_prev = self.position

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in radians. """
        self.desired = position
        self.dirty = True
        msg = self.name + ': new desired position set - ' + str(position)
        rospy.logdebug(msg)

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.controller and self.controller.active():
            # Under and action control, do not interfere
            return
        else:
            self.dirty = True
            self.desired = req.data
 
""" Class to receive Twist commands and publish Odometry data """
class JointController:
    def __init__(self, arduino, joint_dict, controller_name):
        self.arduino = arduino
        self.joint_dict = joint_dict
        self.controller_name = controller_name
        
        self.joints = list()
        self.joint_names = list()
        for name in rospy.get_param(self.joint_dict, dict()).keys():
            self.joints.append(HobbyServo(self, name))
            self.joint_names.append(name)

        # parameters: throttle rate and geometry
        self.rate = rospy.get_param('~joint_controller_rate', 10.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # action server
        self.name = self.controller_name + '/follow_joint_trajectory'
        self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)

        # good old trajectory
        rospy.Subscriber(self.name+'/command', JointTrajectory, self.commandCb)
        self.executing = False

        msg = self.name +': FollowController started'
        rospy.loginfo(msg)
        self.server.start()
        
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        
    def scaleGripperRadiansToMeters(self, input):
        gripper_width_m = 0.036
        gripper_width_deg = 135.0
        in_closed = -radians(self.gripper_width_deg)/2
        in_open = radians(self.gripper_width_deg)/2
        out_closed = self.gripper_width_m/2
        out_open = 0
        return ((input - in_closed) * (out_open - out_closed) / (in_open - in_closed) + out_closed)

    def poll(self):
        if rospy.Time.now() > self.t_next:
            # Read Servo Position
            for joint in self.joints:
                joint.setCurrentFeedback(self.arduino.servo_read(joint.id))

            # Write Servo Position
            for joint in self.joints: 
                if joint.dirty:
                    self.arduino.servo_write(joint.id, joint.desired)
                    joint.dirty = False
                    msg = joint.name + ': position set - ' + str(joint.desired)
                    rospy.logdebug(msg)

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list()
            msg.position = list()
            msg.velocity = list()
            for joint in self.joints:
                msg.name.append(joint.name)
                if (joint.name != 'left_gripper_joint') and (joint.name != 'right_gripper_joint'):
                    msg.position.append(joint.position)
                # Publish gripper joint as distance, not radian
                else:
                    msg.position.append(self.scaleGripperRadiansToMeters(joint.position))
                msg.velocity.append(joint.velocity)
            self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta

    def actionCb(self, goal):
        msg = self.name + ': Action goal recieved.'
        rospy.loginfo(msg)
        traj = goal.trajectory

        if set(self.joint_names) != set(traj.joint_names):
            for j in self.joint_names:
                if j not in traj.joint_names:
                    msg = 'Trajectory joint: \n' + str(j) + '\n does not match action controlled joints: \n' + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
            msg = 'Extra joints in trajectory'
            rospy.logwarn(msg)
            return

        if not traj.points:
            msg = 'Trajectory empy.'
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joint_names]
        except ValueError as val:
            msg = 'Trajectory invalid.'
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text='Execution failed.')

        msg = self.name + ': Trajectory execution complete.'
        rospy.loginfo(msg)

    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            msg = self.name+': Received trajectory, but action is active'
            rospy.loginfo(msg)
            return
        self.executing = True
        self.executeTrajectory(msg)
        self.executing = False    

    def executeTrajectory(self, traj):
        msg = self.name + ': Executing trajectory'
        rospy.loginfo(msg)
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joint_names]
        except ValueError as val:
            msg = 'Invalid joint in trajectory.'
            rospy.logerr(msg)
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        position_prev = [ joint.position for joint in self.joints ]
        for point in traj.points:
            # wait until start
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            # run until end
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,position_prev) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        position_cmd = position_prev[i] + cmd
                        self.joints[i].setControlOutput(position_cmd)
                    else:
                        velocity[i] = 0
                r.sleep()

        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

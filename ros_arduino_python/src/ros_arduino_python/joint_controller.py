#!/usr/bin/env python

"""
    A joint controller class for the Arduino microcontroller
    
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

    def __init__(self, controller, name, ns="~joints"):
        self.controller = controller
        self.name = name
        
        n = ns+"/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.range = rospy.get_param(n+"range", 180)
        
        print "Joint added: " + n + ", id: " + str(self.id) + " range: " + str(self.range)

        self.dirty = False                      # newly updated position?
        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def setCurrentFeedback(self, reading):
        """ Update angle in radians by reading from servo """
        if reading >= -(pi/2) and reading <= (pi/2):     # check validity
            last_angle = self.position
            self.position = reading
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logdebug("Invalid read of servo: id " + str(self.id) + ", value " + str(reading))
            return
        if not self.controller.active:
            self.last_cmd = self.position

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in radians. """
        self.desired = position
        self.dirty = True
        return int(position)

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
    def __init__(self, arduino, controller_side):
        self.arduino = arduino
        self.controller_side = controller_side
        
        self.joints = list()
        for name in rospy.get_param("~joints_"+self.controller_side, dict()).keys():
            self.joints.append(HobbyServo(self, name))

        self.last = rospy.Time.now()

        # parameters: throttle rate and geometry
        self.rate = rospy.get_param("~joint_controller_rate", 10.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # action server
        self.name = self.controller_side+'_arm_controller/follow_joint_trajectory'
        self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)

        # good old trajectory
        rospy.Subscriber(self.name+'/command', JointTrajectory, self.commandCb)
        self.executing = False

        rospy.loginfo("Started FollowController: " + self.name)
        self.server.start()
        
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)


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

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list()
            msg.position = list()
            msg.velocity = list()
            for joint in self.joints:
                msg.name.append(joint.name)
                msg.position.append(joint.position)
                msg.velocity.append(joint.velocity)
            self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")

    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name+": Received trajectory, but action is active")
            return
        self.executing = True
        self.executeTrajectory(msg)
        self.executing = False    

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        last = [ self.joints[joint].position for joint in self.joints ]
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                rospy.logdebug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd
                        self.joints[i].setControlOutput(last[i])
                    else:
                        velocity[i] = 0
                r.sleep()
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

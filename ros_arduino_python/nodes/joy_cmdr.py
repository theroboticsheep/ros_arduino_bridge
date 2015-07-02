#!/usr/bin/env python

import rospy, time
import sys, select, termios, tty
from sensor_msgs.msg import Joy


class JoyCMDR():
    def __init__(self):
        self.msg = """
        ***************************************************
        *                                                 *
        * Open Commander tab for Xbox Controller input    *
        * Controller input published to /joy ros topic    *
        *                                                 *
        * CTRL-C to quit                                  *
        *                                                 *
        ***************************************************
        """

        self.pub = rospy.Publisher('joy', Joy, queue_size=10)
        rospy.init_node('joy_cmdr')
        rospy.loginfo(self.msg)
        
        self.joy_msg = Joy()

        # define the function blocks
        def state_init(self):
            if self.key == '[':
                self.state = 1
        def state_axes(self):
            if self.key == ';':
                self.joy_msg.axes = [float(x) for x in self.line.split(',') if x.strip()]
                self.state = 2
                self.line = ''
                return
            self.line = self.line + self.key
        
        def state_buttons(self):
            if self.key == ']':
                self.joy_msg.buttons = [int(x) for x in self.line.split(',') if x.strip()]
                self.joy_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.joy_msg)
                self.state = 0
                self.line = ''
                return
            self.line = self.line + self.key
        
        # map the inputs to the function blocks
        states = {0 : state_init,
                  1 : state_axes,
                  2 : state_buttons}
        # initial state
        self.state = 0
        self.line = ''
        
        while 1:
            self.key = sys.stdin.read(1)
            if self.key == '\x03':
                return
            # call switch
            states[self.state](self)
            
        rospy.signal_shutdown('Shutting down joy_cmdr node')

if __name__ == '__main__':
    joyCMDR = JoyCMDR()

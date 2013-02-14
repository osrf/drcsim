#!/usr/bin/env python
#
# convert sensor_msgs/Joy messages to osrf_msgs/JointCommands messages

import roslib; roslib.load_manifest('atlas_utils')
import rospy
import sys
import math

from sensor_msgs.msg import Joy
from osrf_msgs.msg import JointCommands

USAGE = 'arm_teleop.py {l|r}'

class ArmTeleop():

    def __init__(self, argv):

        # Prepare a message structure that we'll reuse
        self.atlasJointNames = [ 'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay', 'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax', 'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax', 'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx', 'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']

        if len(argv) != 2:
            self.usage()

        # Depending on which arm we're controlling, note the index into the
        # joint list; we'll do direct control of 6 joints starting there.
        self.num_joints = 6
        if argv[1] == 'l':
            self.idx_offset = self.atlasJointNames.index('atlas::l_arm_usy')
        elif argv[1] == 'r':
            self.idx_offset = self.atlasJointNames.index('atlas::r_arm_usy')
        else:
            self.usage()


        # This stuff (and much else) belongs in a config file
        self.joy_axis_min = 0.0
        self.joy_axis_max = 1.0
        self.joint_min = -math.pi/12.0
        self.joint_max = math.pi/12.0

        self.command = JointCommands()
        self.command.name = list(self.atlasJointNames)
        n = len(self.command.name)
        self.command.position = [0.0] * n
        self.command.velocity = [0.0] * n
        self.command.effort = [0.0] * n
        self.command.kp_position = [0.0] * n
        self.command.ki_position = [0.0] * n
        self.command.kd_position = [0.0] * n
        self.command.kp_velocity = [0.0] * n
        self.command.i_effort_min = [0.0] * n
        self.command.i_effort_max = [0.0] * n

        rospy.init_node('arm_teleop', anonymous=True)
        # now get gains from parameter server
        for i in xrange(len(self.command.name)):
            name = self.command.name[i]
            self.command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
            self.command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
            self.command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
            self.command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
            self.command.i_effort_min[i] = -self.command.i_effort_max[i]

        self.sub = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.pub = rospy.Publisher('atlas/joint_commands', JointCommands)

    def joy_cb(self, msg):
        n = len(self.command.name)
        self.command.position = [0.0] * n
        self.command.velocity = [0.0] * n
        self.command.effort = [0.0] * n
        for i in range(0, self.num_joints):
            position = self.joint_min + ((msg.axes[i] - self.joy_axis_min)/(self.joy_axis_max - self.joy_axis_min)) * (self.joint_max - self.joint_min)
            print '%d: %f (%f)'%(i, position, msg.axes[i])
            self.command.position[self.idx_offset + i] = position
        self.command.header.stamp = rospy.Time.now()
        self.pub.publish(self.command)

    def usage(self):
        print USAGE
        sys.exit(1)


if __name__ == '__main__':
    at = ArmTeleop(rospy.myargv())
    rospy.spin()

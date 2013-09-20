#!/usr/bin/env python
#
# convert sensor_msgs/Joy messages to osrf_msgs/JointCommands messages

import roslib; roslib.load_manifest('drcsim_gazebo')
import rospy
import sys
import math

from sensor_msgs.msg import Joy, JointState
from osrf_msgs.msg import JointCommands

USAGE = 'arm_teleop.py {l|r|lr}'

# Scene button comes last
SCENE_BUTTON_INDEX = -1

class ArmTeleop():

    def __init__(self, argv):

        # We'll keep the latest joint state here
        self.joint_state = None

        # Prepare a message structure that we'll reuse
        self.atlasJointNames = [ 'atlas::back_lbz', 'atlas::back_mby',
                                 'atlas::back_ubx', 'atlas::neck_ay',
                                 'atlas::l_leg_uhz', 'atlas::l_leg_mhx',
                                 'atlas::l_leg_lhy', 'atlas::l_leg_kny',
                                 'atlas::l_leg_uay', 'atlas::l_leg_lax',
                                 'atlas::r_leg_uhz', 'atlas::r_leg_mhx',
                                 'atlas::r_leg_lhy', 'atlas::r_leg_kny',
                                 'atlas::r_leg_uay', 'atlas::r_leg_lax',
                                 'atlas::l_arm_usy', 'atlas::l_arm_shx',
                                 'atlas::l_arm_ely', 'atlas::l_arm_elx',
                                 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
                                 'atlas::r_arm_usy', 'atlas::r_arm_shx',
                                 'atlas::r_arm_ely', 'atlas::r_arm_elx',
                                 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']

        if len(argv) < 2 or len(argv) > 4:
            self.usage()

        # Depending on which arm we're controlling, note the index into the
        # joint list; we'll do direct control of 6 joints starting there.
        self.num_joints = 6
        if argv[1] == 'l':
            self.idx_offset1 = self.atlasJointNames.index('atlas::l_arm_usy')
            self.idx_offset2 = None
        elif argv[1] == 'r':
            self.idx_offset1 = self.atlasJointNames.index('atlas::r_arm_usy')
            self.idx_offset2 = None
        elif argv[1] == 'lr' or argv[1] == 'rl':
            self.idx_offset1 = self.atlasJointNames.index('atlas::l_arm_usy')
            self.idx_offset2 = self.atlasJointNames.index('atlas::r_arm_usy')
        else:
            self.usage()

        # If specified, we'll only do anything when the scene number matches
        self.scene1 = None
        self.scene2 = None
        if len(argv) >= 3:
            self.scene1 = int(argv[2])
        if len(argv) == 4:
            self.scene2 = int(argv[3])

        # This stuff (and much else) belongs in a config file
        self.joy_axis_min = 0.0
        self.joy_axis_max = 1.0
        self.joint_min = -math.pi
        self.joint_max = math.pi

        self.joint_command = JointCommands()
        self.joint_command.name = list(self.atlasJointNames)
        n = len(self.joint_command.name)
        self.joint_command.position = [0.0] * n
        self.joint_command.velocity = [0.0] * n
        self.joint_command.effort = [0.0] * n
        self.joint_command.kp_position = [0.0] * n
        self.joint_command.ki_position = [0.0] * n
        self.joint_command.kd_position = [0.0] * n
        self.joint_command.kp_velocity = [0.0] * n
        self.joint_command.i_effort_min = [0.0] * n
        self.joint_command.i_effort_max = [0.0] * n

        rospy.init_node('arm_teleop', anonymous=True)
        # now get gains from parameter server
        for i in xrange(len(self.joint_command.name)):
            name = self.joint_command.name[i]
            self.joint_command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
            self.joint_command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
            self.joint_command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
            self.joint_command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
            self.joint_command.i_effort_min[i] = -self.joint_command.i_effort_max[i]

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.js_sub = rospy.Subscriber('atlas/joint_states', JointState, self.js_cb)
        self.pub = rospy.Publisher('atlas/joint_commands', JointCommands)

    def js_cb(self, msg):
        self.joint_state = msg

    def joy_cb(self, msg):
        scene1_match = (self.scene1 is None or
                        msg.buttons[SCENE_BUTTON_INDEX] == self.scene1)
        scene2_match = (self.scene2 is None or
                        msg.buttons[SCENE_BUTTON_INDEX] == self.scene2)
        if not scene1_match and not scene2_match:
            return

        # Copy back the latest position state for joints we're not controlling; will drift
        if not self.joint_state:
            return
        self.joint_command.position = list(self.joint_state.position)

        n = len(self.joint_command.name)
        self.joint_command.velocity = [0.0] * n
        self.joint_command.effort = [0.0] * n
        for i in range(self.num_joints):
            position = self.joint_min + ((msg.axes[i] - self.joy_axis_min)/(self.joy_axis_max - self.joy_axis_min)) * (self.joint_max - self.joint_min)
            if self.idx_offset1 != None and scene1_match:
                self.joint_command.position[self.idx_offset1 + i] = position
            if self.idx_offset2 != None and scene2_match:
                self.joint_command.position[self.idx_offset2 + i] = position
        self.joint_command.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_command)

    def usage(self):
        print USAGE
        sys.exit(1)


if __name__ == '__main__':
    at = ArmTeleop(rospy.myargv())
    rospy.spin()

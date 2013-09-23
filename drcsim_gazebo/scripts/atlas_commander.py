#! /usr/bin/env python
import roslib; roslib.load_manifest('drcsim_gazebo')
from keyboard_teleop import AtlasTeleop

from atlas_msgs.msg import WalkDemoAction, \
                           WalkDemoActionGoal, \
                           WalkDemoGoal, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorStepParams, \
                           AtlasBehaviorStandParams, \
                           AtlasBehaviorManipulateParams
from std_msgs.msg import Header

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import actionlib
import math
import rospy
import sys

class AutoAtlasTeleop(AtlasTeleop):

    def init(self):
        # Creates the SimpleActionClient, passing the type of the action
        # () to the constructor.
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', \
          WalkDemoAction)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
          String, None, False, True, None)
        self.commander = rospy.Publisher('/atlas_commander/end', \
          String, None, False, True, None)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("Waiting for atlas/bdi_control")
        self.client.wait_for_server()

    def loginfo(self, str):
        rospy.loginfo(str)

    def debuginfo(self, str):
        rospy.logdebug(str)
 
    def fini(self):
        pass
    
    def dynamic_twist(self, forward, lateral, turn):
        super(AutoAtlasTeleop, self).dynamic_twist(forward, lateral, turn)
        # while last goal is being executed.
        steps = self.build_steps(forward, lateral, turn)
        self.client.wait_for_result(\
           rospy.Duration(self.params["Stride Duration"]["value"] * \
                          len(steps) + 5))

    def run(self, input_keys):
        self.init()
        #while not rospy.is_shutdown():
        for k in input_keys:
            if k[:2] == '--' or k[:2] == '__':
                continue
            self.loginfo("Processing key: " + k)
            self.process_key(k)

        self.commander.publish(True)
        rospy.signal_shutdown("Shutdown")

if __name__ == '__main__':
    rospy.init_node('atlas_commander')
    teleop = AutoAtlasTeleop()
    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        rospy.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    rospy.sleep(5.0)
    print('OK, starting test.')
    if len(sys.argv) < 1:
        print "Parameters needed"
        exit(-1)
    teleop.run(sys.argv[1:])

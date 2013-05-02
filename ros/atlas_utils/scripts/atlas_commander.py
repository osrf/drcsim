#! /usr/bin/env python
import roslib; roslib.load_manifest('atlas_utils')
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

    def fini(self):
        pass

    def run(self, input_keys):
        self.init()
        while not rospy.is_shutdown():
            for k in input_keys:
                rospy.loginfo("Processing key: " + k)
                self.process_key(k)

        self.commander.publish(True)
        rospy.signal_shutdown("Shutdown")

    # For everything that can't be a binding, use if/elif instead
    def process_key(self, ch):
        if self.directions.has_key(ch):
            self.process_movement(ch)
        elif ch == 'r':
            self.reset_to_standing()
        try:
            if (int(ch) >= self.params["Walk Sequence Length"]["min"] and \
                int(ch) <= self.params["Walk Sequence Length"]["max"]):
                self.params["Walk Sequence Length"]["value"] = int(ch)
                rospy.loginfo("Walk Sequence Length: " + \
                  str(self.params["Walk Sequence Length"]["value"]))
        except ValueError:
            pass

if __name__ == '__main__':
    rospy.init_node('atlas_commander')
    teleop = AutoAtlasTeleop()
    if len(argv) < 1:
        print "Parameters needed"
        exit(-1)
    teleop.run(sys.argv)

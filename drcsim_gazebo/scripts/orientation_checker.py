#!/usr/bin/env python
import roslib
roslib.load_manifest('drcsim_gazebo')
import sys
import rospy
import unittest
import rostest
from atlas_msgs.msg import AtlasState
import time

class OrientationChecker(unittest.TestCase):

    def check_standup(self, orientation):
        if (orientation.x < 0.1) and \
           (orientation.y < 0.1) and \
           (orientation.z < 0.1) and \
           (orientation.w > 0.95):
            rospy.loginfo("Stand up")
            self.standup_msgs += 1
            return True
        else:
            rospy.loginfo("Fallen")
            self.result       = False
            self.result_ready = True
            return False

    def callback(self, data):
        rospy.loginfo("Received")
        if self.check_standup(data.orientation) and self.standup_msgs == 1000:
            self.result       = True
            self.result_ready = True

    def test_result(self):
        self.standup_msgs  = 0
        self.result_ready  = False
        self.result        = False

        self.pub = rospy.Subscriber("/atlas/atlas_state", AtlasState, self.callback)

        while not self.result_ready:
            time.sleep(0.1)
            pass

        self.assertTrue(self.result, "Robot is not stand up (orientation not close to (0,0,0,1)")

if __name__ == '__main__':
    rospy.init_node('orientation_checker', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    
    time.sleep(10.0)
    rostest.run('drcsim_gazebo','orientation_checker', OrientationChecker, sys.argv)

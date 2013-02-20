#!/usr/bin/env python

import roslib; #roslib.load_manifest('atlas_utils')
import rospy, math

from std_msgs.msg import String

def demo():
    # Initialize the node
    rospy.init_node('five_steps_demo')

    # Setup the publishers for each joint
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    control_mode = rospy.Publisher('/atlas/control_mode', String, None, False, True, None)

    while mode.get_num_connections() == 0:
      rospy.sleep(0.1)

    mode.publish("harnessed")
    control_mode.publish("stand-prep")
    rospy.sleep(5.0)
    mode.publish("nominal")
    rospy.sleep(0.3)
    control_mode.publish("stand")
    rospy.sleep(1.0)
    control_mode.publish("walk")

if __name__ == '__main__':
    try:
        demo()
    except rospy.ROSInterruptException: pass

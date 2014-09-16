#!/usr/bin/env python
# Do not use, testing only
import roslib; roslib.load_manifest('drcsim_gazebo')
import rospy, math

from atlas_msgs.msg import Test

def jointStateCommand():
    # Initialize the node
    rospy.init_node('test')

    # Setup the publishers for each joint
    pub = rospy.Publisher('/atlas/debug/test', Test, None, False, True, None)

    while pub.get_num_connections() == 0:
      rospy.sleep(0.1)

    test = Test();
    test.damping = [
            # lbz mby ubx
            10000, 5000, 5000, 
            # l_arm elx ely mwx shx usy uwy
            1.0, 1.0, 1.0, 10.0, 10.0, 1.0,
            # l_leg kny lax lhy mhx uay uhz
            1.0, 50.0, 10.0, 10.0, 100.0, 10.0,
            # neck 
            1.0,
            # r_arm elx ely mwx shx usy uwy
            1.0, 1.0, 1.0, 10.0, 10.0, 1.0,
            # r_leg kny lax lhy mhx uay uhz
            1.0, 50.0, 10.0, 10.0, 100.0, 10.0]
                    
    test.kp_position = [
            # lbz mby ubx
            2000.0, 4000.0, 2000.0,
            # l_arm elx ely mwx shx usy uwy
            200.0, 200.0, 100.0, 1000.0, 2000.0, 50.0,
            # l_leg kny lax lhy mhx uay uhz
            1000.0, 300.0, 2000.0, 2000.0, 900.0, 2000.0,
            # neck 
            20.0,
            # r_arm elx ely mwx shx usy uwy
            200.0, 200.0, 100.0, 1000.0, 2000.0, 50.0,
            # r_leg kny lax lhy mhx uay uhz
            1000.0, 300.0, 2000.0, 2000.0, 900.0, 2000.0]

    test.kd_position = [
            # lbz mby ubx
            100.0, 100.0, 100.0,
            # l_arm elx ely mwx shx usy uwy
            3.0, 3.0, 0.2, 20.0, 3.0, 0.1,
            # l_leg kny lax lhy mhx uay uhz
            10.0, 1.0, 10.0, 10.0, 2.0, 10.0,
            # neck 
            1.0,
            # r_arm elx ely mwx shx usy uwy
            3.0, 3.0, 0.2, 20.0, 3.0, 0.1,
            # r_leg kny lax lhy mhx uay uhz
            10.0, 1.0, 10.0, 10.0, 2.0, 10.0]

    pub.publish(test)

if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

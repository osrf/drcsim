#!/usr/bin/env python

import roslib; roslib.load_manifest('drcsim_gazebo')
import rospy, math

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

def jointStateCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    pose = rospy.Publisher('/pose', Pose)
    while pose.get_num_connections() == 0:
      rospy.sleep(0.1)

    # Setup the publishers for each joint
    back_bkz = rospy.Publisher('/back_bkz_position_controller/command', Float64)
    back_bky = rospy.Publisher('/back_bky_position_controller/command', Float64)
    back_bkx = rospy.Publisher('/back_bkx_position_controller/command', Float64)
    l_arm_elx = rospy.Publisher('/l_arm_elx_position_controller/command', Float64)
    l_arm_ely = rospy.Publisher('/l_arm_ely_position_controller/command', Float64)
    l_arm_wrx = rospy.Publisher('/l_arm_wrx_position_controller/command', Float64)
    l_arm_shx = rospy.Publisher('/l_arm_shx_position_controller/command', Float64)
    l_arm_shy = rospy.Publisher('/l_arm_shy_position_controller/command', Float64)
    l_arm_wry = rospy.Publisher('/l_arm_wry_position_controller/command', Float64)
    l_f0_j0 = rospy.Publisher('/l_f0_j0_position_controller/command', Float64)
    l_f0_j1 = rospy.Publisher('/l_f0_j1_position_controller/command', Float64)
    l_f0_j2 = rospy.Publisher('/l_f0_j2_position_controller/command', Float64)
    l_f1_j0 = rospy.Publisher('/l_f1_j0_position_controller/command', Float64)
    l_f1_j1 = rospy.Publisher('/l_f1_j1_position_controller/command', Float64)
    l_f1_j2 = rospy.Publisher('/l_f1_j2_position_controller/command', Float64)
    l_f2_j0 = rospy.Publisher('/l_f2_j0_position_controller/command', Float64)
    l_f2_j1 = rospy.Publisher('/l_f2_j1_position_controller/command', Float64)
    l_f2_j2 = rospy.Publisher('/l_f2_j2_position_controller/command', Float64)
    l_f3_j0 = rospy.Publisher('/l_f3_j0_position_controller/command', Float64)
    l_f3_j1 = rospy.Publisher('/l_f3_j1_position_controller/command', Float64)
    l_f3_j2 = rospy.Publisher('/l_f3_j2_position_controller/command', Float64)
    l_leg_kny = rospy.Publisher('/l_leg_kny_position_controller/command', Float64)
    l_leg_akx = rospy.Publisher('/l_leg_akx_position_controller/command', Float64)
    l_leg_hpy = rospy.Publisher('/l_leg_hpy_position_controller/command', Float64)
    l_leg_hpx = rospy.Publisher('/l_leg_hpx_position_controller/command', Float64)
    l_leg_aky = rospy.Publisher('/l_leg_aky_position_controller/command', Float64)
    l_leg_hpz = rospy.Publisher('/l_leg_hpz_position_controller/command', Float64)
    neck_ry = rospy.Publisher('/neck_ry_position_controller/command', Float64)
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
    r_arm_wrx = rospy.Publisher('/r_arm_wrx_position_controller/command', Float64)
    r_arm_shx = rospy.Publisher('/r_arm_shx_position_controller/command', Float64)
    r_arm_shy = rospy.Publisher('/r_arm_shy_position_controller/command', Float64)
    r_arm_wry = rospy.Publisher('/r_arm_wry_position_controller/command', Float64)
    r_f0_j0 = rospy.Publisher('/r_f0_j0_position_controller/command', Float64)
    r_f0_j1 = rospy.Publisher('/r_f0_j1_position_controller/command', Float64)
    r_f0_j2 = rospy.Publisher('/r_f0_j2_position_controller/command', Float64)
    r_f1_j0 = rospy.Publisher('/r_f1_j0_position_controller/command', Float64)
    r_f1_j1 = rospy.Publisher('/r_f1_j1_position_controller/command', Float64)
    r_f1_j2 = rospy.Publisher('/r_f1_j2_position_controller/command', Float64)
    r_f2_j0 = rospy.Publisher('/r_f2_j0_position_controller/command', Float64)
    r_f2_j1 = rospy.Publisher('/r_f2_j1_position_controller/command', Float64)
    r_f2_j2 = rospy.Publisher('/r_f2_j2_position_controller/command', Float64)
    r_f3_j0 = rospy.Publisher('/r_f3_j0_position_controller/command', Float64)
    r_f3_j1 = rospy.Publisher('/r_f3_j1_position_controller/command', Float64)
    r_f3_j2 = rospy.Publisher('/r_f3_j2_position_controller/command', Float64)
    r_leg_kny = rospy.Publisher('/r_leg_kny_position_controller/command', Float64)
    r_leg_akx = rospy.Publisher('/r_leg_akx_position_controller/command', Float64)
    r_leg_hpy = rospy.Publisher('/r_leg_hpy_position_controller/command', Float64)
    r_leg_hpx = rospy.Publisher('/r_leg_hpx_position_controller/command', Float64)
    r_leg_aky = rospy.Publisher('/r_leg_aky_position_controller/command', Float64)
    r_leg_hpz = rospy.Publisher('/r_leg_hpz_position_controller/command', Float64)

    # Set configuration
    while not rospy.is_shutdown():
        rospy.sleep(3.0)

        t = 6 * rospy.get_time()
        x = 0.0 * math.sin(t)

        p = Pose()
        p.position.x = 0
        p.position.y = 0
        p.position.z = 0.93
        p.orientation.w = 1
        p.orientation.x = 0
        p.orientation.y = 0
        p.orientation.z = 0
        pose.publish(p)

        back_bkz.publish(0.0)
        back_bky.publish(x)
        back_bkx.publish(0.0)
        l_arm_elx.publish(0.0)
        l_arm_ely.publish(0.0)
        l_arm_wrx.publish(0.0)
        l_arm_shx.publish(0.0)
        l_arm_shy.publish(0.0)
        l_arm_wry.publish(0.0)
        l_f0_j0.publish(0.0)
        l_f0_j1.publish(0.0)
        l_f0_j2.publish(0.0)
        l_f1_j0.publish(0.0)
        l_f1_j1.publish(0.0)
        l_f1_j2.publish(0.0)
        l_f2_j0.publish(0.0)
        l_f2_j1.publish(0.0)
        l_f2_j2.publish(0.0)
        l_f3_j0.publish(0.0)
        l_f3_j1.publish(0.0)
        l_f3_j2.publish(0.0)
        l_leg_kny.publish(0.0)
        l_leg_akx.publish(0.0)
        l_leg_hpy.publish(0.0)
        l_leg_hpx.publish(0.0)
        l_leg_aky.publish(0.0)
        l_leg_hpz.publish(0.0)
        neck_ry.publish(0.0)
        r_arm_elx.publish(0.0)
        r_arm_ely.publish(0.0)
        r_arm_wrx.publish(0.0)
        r_arm_shx.publish(0.0)
        r_arm_shy.publish(0.0)
        r_arm_wry.publish(0.0)
        r_f0_j0.publish(0.0)
        r_f0_j1.publish(0.0)
        r_f0_j2.publish(0.0)
        r_f1_j0.publish(0.0)
        r_f1_j1.publish(0.0)
        r_f1_j2.publish(0.0)
        r_f2_j0.publish(0.0)
        r_f2_j1.publish(0.0)
        r_f2_j2.publish(0.0)
        r_f3_j0.publish(0.0)
        r_f3_j1.publish(0.0)
        r_f3_j2.publish(0.0)
        r_leg_kny.publish(0.0)
        r_leg_akx.publish(0.0)
        r_leg_hpy.publish(0.0)
        r_leg_hpx.publish(0.0)
        r_leg_aky.publish(0.0)
        r_leg_hpz.publish(0.0)

if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

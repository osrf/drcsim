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
    back_lbz = rospy.Publisher('/back_lbz_position_controller/command', Float64)
    back_mby = rospy.Publisher('/back_mby_position_controller/command', Float64)
    back_ubx = rospy.Publisher('/back_ubx_position_controller/command', Float64)
    l_arm_elx = rospy.Publisher('/l_arm_elx_position_controller/command', Float64)
    l_arm_ely = rospy.Publisher('/l_arm_ely_position_controller/command', Float64)
    l_arm_mwx = rospy.Publisher('/l_arm_mwx_position_controller/command', Float64)
    l_arm_shx = rospy.Publisher('/l_arm_shx_position_controller/command', Float64)
    l_arm_usy = rospy.Publisher('/l_arm_usy_position_controller/command', Float64)
    l_arm_uwy = rospy.Publisher('/l_arm_uwy_position_controller/command', Float64)
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
    l_leg_lax = rospy.Publisher('/l_leg_lax_position_controller/command', Float64)
    l_leg_lhy = rospy.Publisher('/l_leg_lhy_position_controller/command', Float64)
    l_leg_mhx = rospy.Publisher('/l_leg_mhx_position_controller/command', Float64)
    l_leg_uay = rospy.Publisher('/l_leg_uay_position_controller/command', Float64)
    l_leg_uhz = rospy.Publisher('/l_leg_uhz_position_controller/command', Float64)
    neck_ay = rospy.Publisher('/neck_ay_position_controller/command', Float64)
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
    r_arm_mwx = rospy.Publisher('/r_arm_mwx_position_controller/command', Float64)
    r_arm_shx = rospy.Publisher('/r_arm_shx_position_controller/command', Float64)
    r_arm_usy = rospy.Publisher('/r_arm_usy_position_controller/command', Float64)
    r_arm_uwy = rospy.Publisher('/r_arm_uwy_position_controller/command', Float64)
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
    r_leg_lax = rospy.Publisher('/r_leg_lax_position_controller/command', Float64)
    r_leg_lhy = rospy.Publisher('/r_leg_lhy_position_controller/command', Float64)
    r_leg_mhx = rospy.Publisher('/r_leg_mhx_position_controller/command', Float64)
    r_leg_uay = rospy.Publisher('/r_leg_uay_position_controller/command', Float64)
    r_leg_uhz = rospy.Publisher('/r_leg_uhz_position_controller/command', Float64)

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

        back_lbz.publish(0.0)
        back_mby.publish(x)
        back_ubx.publish(0.0)
        l_arm_elx.publish(0.0)
        l_arm_ely.publish(0.0)
        l_arm_mwx.publish(0.0)
        l_arm_shx.publish(0.0)
        l_arm_usy.publish(0.0)
        l_arm_uwy.publish(0.0)
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
        l_leg_lax.publish(0.0)
        l_leg_lhy.publish(0.0)
        l_leg_mhx.publish(0.0)
        l_leg_uay.publish(0.0)
        l_leg_uhz.publish(0.0)
        neck_ay.publish(0.0)
        r_arm_elx.publish(0.0)
        r_arm_ely.publish(0.0)
        r_arm_mwx.publish(0.0)
        r_arm_shx.publish(0.0)
        r_arm_usy.publish(0.0)
        r_arm_uwy.publish(0.0)
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
        r_leg_lax.publish(0.0)
        r_leg_lhy.publish(0.0)
        r_leg_mhx.publish(0.0)
        r_leg_uay.publish(0.0)
        r_leg_uhz.publish(0.0)

if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

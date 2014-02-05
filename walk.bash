
# back_bkz , back_bky , back_bkx , neck_ry  , l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_aky, l_leg_akx, r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_aky, r_leg_akx, l_arm_shy, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, r_arm_shy, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx

# standard
rosservice call /atlas/set_joint_damping '{damping_coefficients: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'

#alias set_joint_damping (hpz = 20)
rosservice call /atlas/set_joint_damping 'damping_coefficients: [0.1, 0.1, 0.1, 0.1, 20.0, 0.1, 0.1, 0.1, 0.1, 0.1, 20.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]'

#alias set_joint_damping (hpz = 20, reset = 1.0)
rosservice call /atlas/set_joint_damping 'damping_coefficients: [1.0, 1.0, 1.0, 1.0, 20.0, 1.0, 1.0, 1.0, 1.0, 1.0, 20.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]'

#alias set_joint_damping (all = 0.01)
rosservice call /atlas/set_joint_damping '{damping_coefficients: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]}'

#set_joint_damping_2
rosservice call /atlas/set_joint_damping 'damping_coefficients: [20.0, 20.0, 20.0, 20.0,    1.0, 1.0, 20.0, 1.0, 1.0, 1.0,    1.0, 1.0, 20.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]'
# set_joint_damping_torso
rosservice call /atlas/set_joint_damping 'damping_coefficients: [20.0, 20.0, 20.0, 20.0,    1.0, 1.0, 20.0, 1.0, 1.0, 1.0,    1.0, 1.0, 20.0, 1.0, 1.0, 1.0,      80.0, 80.0, 80.0, 80.0, 80.0, 80.0,        80.0, 80.0, 80.0, 80.0, 80.0, 80.0]'


#!/bin/bash

#   killall roslaunch
#   killall gzserver
#  
#   export VRC_CHEATS_ENABLED=1
#  
#   roslaunch drcsim_gazebo grasp_exp.launch &
#  
#   sleep 18
#  
#   echo pinning
#   rostopic pub --once /atlas/mode std_msgs/String -- pinned_with_gravity
#  
#   sleep 3
#  
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]}'
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}'
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]}'
#   rosservice call /sandia_hands/set_joint_damping '{damping_coefficients: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]}'

rosservice call /atlas/set_joint_damping "damping_coefficients: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]"
rostopic pub --once /atlas/joint_commands osrf_msgs/JointCommands '{ position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] }'
sleep 3

gz model --delete -m beer_heavy

rostopic pub --once /sandia_hands/r_hand/joint_commands osrf_msgs/JointCommands "{name: ['f0_j0', 'f0_j1', 'f0_j2', 'f1_j0', 'f1_j1', 'f1_j2', 'f2_j0', 'f2_j1', 'f2_j2', 'f3_j0', 'f3_j1', 'f3_j2'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0],  kp_position: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
rosservice call /sandia_hands/r_hand/simple_grasp ' { grasp: { name: "cylindrical", closed_amount: 0.5 } }'

sleep 10

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: 0},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 5

# gz model -f ~/.gazebo/models/cordless_drill/model.sdf -m beer_heavy -x -0.303 -y 0.686 -z 1.417 -R 0 -P 0 -Y 1.57
gz model -f ~/.gazebo/models/beer_heavy/model.sdf -m beer_heavy -x 0.063191 -y -1.016631 -z 1.517 -R 0 -P 0 -Y 1.57 # spawn model
# gz model -m beer_heavy -x 0.063191 -y -1.016631 -z 1.517 -R 0 -P 0 -Y 1.57 # move model only

sleep 1

# rosservice call /sandia_hands/r_hand/simple_grasp ' { grasp: { name: "cylindrical", closed_amount: 10.0 } }'
rostopic pub --once /sandia_hands/r_hand/joint_commands osrf_msgs/JointCommands "{name: ['f0_j0', 'f0_j1', 'f0_j2', 'f1_j0', 'f1_j1', 'f1_j2', 'f2_j0', 'f2_j1', 'f2_j2', 'f3_j0', 'f3_j1', 'f3_j2'], position: [0.0, 4.0, 2.0, 0.0, 4.0, 2.0, 0.0, 4.0, 2.0, 0.2, 4.0, 2.0],  kp_position: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"

sleep 5

rostopic pub --once /atlas/joint_commands osrf_msgs/JointCommands '{ position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.8] }'

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -1.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 2
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -3.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 2
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -5.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 2
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -7.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 2
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -9.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 150, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 3

echo start plotting!

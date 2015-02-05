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

rosservice call /atlas/set_joint_damping "damping_coefficients: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]" &
rostopic pub --once /atlas/joint_commands osrf_msgs/JointCommands '{ position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] }' &
sleep 1

gz model --delete -m beer_heavy

# open right hand
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0} &

sleep 3

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: 0},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 1

gz model -f ~/.gazebo/models/cordless_drill/model.sdf -m beer_heavy -x 0.109294 -y -1.212146 -z 1.271513 -R 0 -P 0 -Y 1.57
# gz model -f ~/.gazebo/models/beer_heavy/model.sdf -m beer_heavy -x 0.12969 -y -1.20248 -z 1.386611 -R 0 -P 0 -Y 1.57 # spawn model

sleep 1

rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0} &

sleep 5

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -1.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 1
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -3.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 1
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -5.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 1
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -7.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"
sleep 1
rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -9.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 3

echo start plotting!

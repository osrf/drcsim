#!/bin/bash

killall roslaunch
killall gzserver

export VRC_CHEATS_ENABLED=1

roslaunch drcsim_gazebo grasp_exp.launch &

sleep 18

echo damping $1

echo pinning
rostopic pub --once /atlas/mode std_msgs/String -- pinned_with_gravity

sleep 3

rostopic pub --once /atlas/joint_commands osrf_msgs/JointCommands '{ position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5] }'

sleep 3

rosservice call /sandia_hands/r_hand/simple_grasp ' { grasp: { name: "cylindrical", closed_amount: 0.7 } }'

sleep 10

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: 0},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 5

gzfactory spawn -f ~/.gazebo/models/cordless_drill/model.sdf -x -0.303 -y 0.686 -z 1.417 -R 0 -P 0 -Y 1.57

sleep 1

rosservice call /sandia_hands/r_hand/simple_grasp ' { grasp: { name: "cylindrical", closed_amount: 0.9 } }'

if [ 'x$1' != 'x' ] ; then
echo set joint damping $1
rosservice call /sandia_hands/set_joint_damping "{ damping_coefficients: [$1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1, $1] }"
fi

sleep 3

rosservice call /gazebo/set_physics_properties "{ time_step: 0.001, max_update_rate: 1000, gravity: {z: -9.8},  ode_config: {auto_disable_bodies: 0, sor_pgs_iters: 50, sor_pgs_w: 1.4, contact_max_correcting_vel: 100, erp: 0.2 } }"

sleep 3

echo start plotting!

#!/bin/bash

#recipient="nate@osrfoundation.org"

while [ 1 ]; do
  task_num=`expr $RANDOM % 15 + 1`
  task="vrc_final_task$task_num"

  roslaunch vrc_finals ${task}.launch &

  sleep 20
  run_id=`rosparam get /run_id`

  # TODO: Add in task specific commands that move atlas

  # Sleep for 60 minutes
  sleep 3600
  gzlog stop
  sleep 20
  killall -INT roslaunch
  sleep 15
  num_end_tags=`grep '</gazebo_log>' /tmp/${task}/state.log | wc | awk {'print $1'}`

  if [ $num_end_tags -ne 1 ]; then
    echo "[FAIL] Wrong number of end tags: $num_end_tags" >> ~/.ros/log/$run_id/$task.log
  else
    echo "[PASS] Correct number of end tags: $num_end_tags" >> ~/.ros/log/$run_id/$task.log
  fi

  last_line=`tail -n 1 /tmp/${task}/state.log`
  if [ $last_line != '</gazebo_log>' ]; then
    echo "[FAIL] Wrong last line: $last_line" >> ~/.ros/log/$run_id/$task.err
  else
    echo "[PASS] Correct last line: $last_line" >> ~/.ros/log/$run_id/$task.log
  fi
done

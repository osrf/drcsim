source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=~/projects/osrf/drcsim/ros:${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH=~/local/share/drc_sim-0.1.0::${ROS_PACKAGE_PATH}

export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/gazebo_models
export GAZEBO_RESOURCE_PATH=~/local/share/gazebo-1.2.0
export GAZEBO_PLUGIN_PATH=~/local/lib/gazebo-1.2.0/plugins
export OGRE_RESOURCE_PATH=~/local/lib/OGRE
export PATH=~/local/bin:${PATH}
export LD_LIBRARY_PATH=~/local/lib:${LD_LIBRARY_PATH}

export GAZEBO_RESOURCE_PATH=~/local/share/drc_sim-0.1.0:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/local/lib/drc_sim-0.1.0/plugins:${GAZEBO_PLUGIN_PATH}


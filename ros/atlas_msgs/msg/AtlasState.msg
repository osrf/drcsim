# Everything a robot needs
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance

geometry_msgs/Wrench l_foot
geometry_msgs/Wrench r_foot
geometry_msgs/Wrench l_hand
geometry_msgs/Wrench r_hand

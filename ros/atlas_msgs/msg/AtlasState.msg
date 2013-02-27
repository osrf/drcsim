# testing everything a robot needs
Header header

int32 back_lbz  = 0
int32 back_mby  = 1
int32 back_ubx  = 2
int32 neck_ay   = 3
int32 l_leg_uhz = 4
int32 l_leg_mhx = 5
int32 l_leg_lhy = 6
int32 l_leg_kny = 7
int32 l_leg_uay = 8
int32 l_leg_lax = 9
int32 r_leg_uhz = 10
int32 r_leg_mhx = 11
int32 r_leg_lhy = 12
int32 r_leg_kny = 13
int32 r_leg_uay = 14
int32 r_leg_lax = 15
int32 l_arm_usy = 16
int32 l_arm_shx = 17
int32 l_arm_ely = 18
int32 l_arm_elx = 19
int32 l_arm_uwy = 20
int32 l_arm_mwx = 21
int32 r_arm_usy = 22
int32 r_arm_shx = 23
int32 r_arm_ely = 24
int32 r_arm_elx = 25
int32 r_arm_uwy = 26
int32 r_arm_mwx = 27

int32[] joint_index

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

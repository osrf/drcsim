# For Setting MultiStep Walking Commands
Header header

# to be filled from pos_est
geometry_msgs/Vector3[] pelvis_position  # Position estimate of robot pelvis
                                         # (x,y,z) in meters
geometry_msgs/Vector3[] pelvis_velocity  # Velocity estimate of robot pelvis
                                         # (xd,yd,zd) in meters per second
geometry_msgs/Vector3[2] foot_pos_est    # World position estimate for feet
                                         # 0 - left, 1 - right
uint32 curent_step_index                 # Current step index in multi_step
                                         # or single_step modes.


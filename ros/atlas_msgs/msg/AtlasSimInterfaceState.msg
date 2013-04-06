# Feedback from AtlasSimInterface Controller after calling process_control_input
int32 error_code                         # error code returned by
                                         # process_control_input.
                                         # See AtlasSimInterfaceTypes.h
                                         # AtlasErrorCode
                                         # for list of enums.
geometry_msgs/Vector3 pelvis_position    # Position estimate of robot pelvis
                                         # (x,y,z) in meters
geometry_msgs/Vector3 pelvis_velocity    # Velocity estimate of robot pelvis
                                         # (xd,yd,zd) in meters per second
geometry_msgs/Pose[2] foot_pose_est      # World position estimate for feet
                                         # 0 - left, 1 - right
uint32 current_step_index                # Current step index in multi_step
                                         # or single_step modes.


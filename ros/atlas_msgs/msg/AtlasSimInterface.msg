# For Setting MultiStep Walking Commands
Header header

# Takes this many steps and switch to stand mode
uint32 steps

# permissible values for foot_index
uint32 LEFT_FOOT = 0
uint32 RIGHT_FOOT = 1
# permissible values for behavior
uint32 USER              =  0 # disable AtlasSimInterface updates, rely on
                              # /atlas/atlas_command or /atlas/joint_commands
uint32 PID_OFF           =  1 # for convenience, option to turn off PID gains
uint32 RELOAD_PID        =  2 # for convenience, option to reload PID gains
                              # from ROS params
uint32 FREEZE            =  3 # safety mode (new?)
uint32 STAND_PREP        =  4 # stand-prep (AtlasSimInterface documentation)
uint32 WALK              =  5 # multi-step walk
uint32 STEP              =  6 # single step walk
uint32 STAND             =  7 # stand (no manipulate?)
uint32 MANIPULATE        =  8 # stand and allows manipulation.
uint32 RESET             =  9 # see AtlasSimInterface.h for documentation.
uint32 DEMO1             = 10 # walk in straight line
uint32 DEMO2             = 11 # figure 8

time start_time     	       # Start time for step trajectory
uint32 behavior              # can be one of enums above

# multi_step walking trajectory parameters
atlas_msgs/AtlasBehaviorStepData[] walk_params

# parameters for single_step behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStepData step_params

# parameters for standing behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStandParams stand_params


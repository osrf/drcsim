# For Setting MultiStep Walking Commands
Header header

# Takes this many steps and switch to stand mode
uint32 steps

# permissible values for foot_index
uint32 LEFT_FOOT = 0
uint32 RIGHT_FOOT = 1
# permissible values for behavior
uint32 NONE              =  0 # disable AtlasSimInterface updates, rely on
                              # /atlas/atlas_command or /atlas/joint_commands
uint32 PID_OFF           =  1 # for convenience, option to turn off PID gains
uint32 RELOAD_PID        =  2 # for convenience, option to reload PID gains
                              # from ROS params
uint32 SAFETY            =  3 # safety mode
uint32 STAND_PREP        =  4 # stand-prep (AtlasSimInterface documentation)
uint32 MULTI_STEP_WALK   =  5 # multi-step walk
uint32 SINGLE_STEP_WALK  =  6 # single step walk
uint32 STAND             =  7 # stand (and manipulate)
uint32 RESET             =  8 # see AtlasSimInterface.h for documentation.
uint32 DEMO1             =  9 # walk in straight line
uint32 DEMO2             = 10 # figure 8

time start_time     	       # Start time for step trajectory

uint32 behavior              # can be one of MULTI_STEP_WALK,
                             # SINGLE_STEP_WALK, STAND, NONE

# multi_step walking trajectory parameters
atlas_msgs/AtlasBehaviorStepParams[] multistep_walk_params

# parameters for single_step behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStepParams singlestep_walk_params

# parameters for standing behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStandParams stand_params


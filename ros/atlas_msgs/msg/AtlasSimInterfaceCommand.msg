# For Setting MultiStep Walking Commands
Header header

# permissible values for behavior
int32 USER              =  3 # disable AtlasSimInterface updates, rely on
                             # /atlas/atlas_command or /atlas/joint_commands
int32 STAND             =  4 # stand (no manipulate?)
int32 FREEZE            =  5 # safety mode (new?)
int32 STAND_PREP        =  6 # stand-prep (AtlasSimInterface documentation)
int32 WALK              =  7 # multi-step walk
int32 STEP              =  8 # single step walk
int32 MANIPULATE        =  9 # stand and allows manipulation.
int32 DEMO1             = 10 # walk in straight line
int32 DEMO2             = 11 # figure 8

int32 behavior                # can be one of
                              # USER, FREEZE, STAND_PREP
                              # WALK, STEP, STAND, MANIPULATE
                              # DEMO1, DEMO2

# multi_step walking trajectory parameters
atlas_msgs/AtlasBehaviorStepData[] walk_params

# parameters for single_step behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStepData step_params

# parameters for standing behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStandParams stand_params

# same k_effort as AtlasCommand
uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.


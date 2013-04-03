# For Setting MultiStep Walking Commands
Header header

# Takes this many steps and switch to stand mode
uint32 steps

# permissible values for foot_index
uint32 LEFT_FOOT = 0
uint32 RIGHT_FOOT = 1
# permissible values for behavior
uint32 MULTI_STEP_WALK = 0
uint32 SINGLE_STEP_WALK = 1
uint32 STAND = 2

time start_time     	           # Start time for step trajectory

uint32 behaviors                 # can be one of MULTI_STEP_WALK, SINGLE_STEP_WALK or STAND

# multi_step walking trajectory parameters
atlas_msgs/AtlasBehaviorMultiStepParams multistep_walk_params

# parameters for single_step behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStepParams singlestep_walk_params

# parameters for standing behavior (not yet implemented)
atlas_msgs/AtlasBehaviorStandParams stand_params


# Command to move the HANDLE hand
# 
# WARNING:
# Not all control types supported yet.
# Ideally, this would be a service call, but that functionality does not exist
# yet.

# The different control types
uint8 VELOCITY = 1
uint8 POSITION = 2
uint8 CURRENT  = 3
uint8 VOLTAGE  = 4

# The control type for each motor.
# Note, the spread motor does not support these control schemes.
int32[5] type

# The value to set.
float32[5] value

# Whether or not to control each motor.
bool[5] valid

# To be added if/when this becomes a service call:
#---
#bool ok
#string reason

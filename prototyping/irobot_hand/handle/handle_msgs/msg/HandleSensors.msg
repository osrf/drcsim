# All of the sensors on the HANDLE hand.
#
# NOTE: In general, the order of the arrays is: [F1, F2, F3, F3 Ant., Spread].
# Where: F1 is analogous to your index finger, F2 is analogous to your 
# middle finger, F3 is analogous to your thumb, F3 Ant. is the antagonistic 
# motor for F3, and Spread is the motor which controls the rotation of F1 and
# F2.
#

# Currently only used for time stamp.  Time stamp set from Overo's clock.
# Not guaranteed to be in sync with current time.
Header header

# The hall effect sensor on the finger motors.  
# 24 counts per motor revolution x 300 revolutions for one full finger motion.
# (Fully open to fully closed).
# [F1, F2, F3, F3 Ant.]
int32[4] motorHallEncoder

# The temperature of the finger motor windings, in Celsius.
# [F1, F2, F3, F3 Ant.]
float32[4] motorWindingTemp

# The air temperature as measured inside the housing, in Celsius.
float32 airTemp

# The level of the 33 Volt supply.
#float32 volts33

# The level of the 12 Volt supply.
#float32 volts12

# The level of the 48 Volt supply.
#float32 volts48

# The motor velocity in RPM.  (Hall encoder ticks per minute)
# [F1, F2, F3, F3 Ant.]
int32[4] motorVelocity

# The temperature of the motor housing, in Celsius.
# [F1, F2, F3, F3 Ant., Spread]
float32[5] motorHousingTemp

# The motor current.
# [F1, F2, F3, F3 Ant., Spread]
float32[5] motorCurrent

# The tactile array for each finger.  In units of kPa.
# [F1, F2, F3]
# Note there are 12 proximal and 10 distal sensors.
Finger[3] fingerTactile

# The tactile array for the palm.  In units of kPa.
float32[48] palmTactile

# The encoder on the F1 / F2 rotation.
# Approx. 768 ticks to rotate the fingers 90 degrees.
int32 fingerSpread

# The proximal joint angle.  Approx 2.84 ticks per degree.
# [F1, F2, F3]
int32[3] proximalJointAngle

# The finger distal joint flexture angle
# [F1, F2, F3]
# Note there are 4 readings on either side of the joint.
Finger[3] distalJointAngle

# The cable tension in F1 and F2.
# [F1, F2]
#CableTension[2] cableTension

# The dynamic PVDF sensors in each finger.
# [F1, F2, F3]
# Note there are 3 sensors on each finger link.
#Finger[3] fingerPVDF

# The dynamic PVDF sensors in the palm.
#float32[4] palmPVDF

# The accelerometer in each finger.  units?  axes?
# [F1, F2, F3]
geometry_msgs/Vector3[3] fingerAcceleration

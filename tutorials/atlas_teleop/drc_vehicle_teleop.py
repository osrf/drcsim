#!/usr/bin/env python

import roslib
roslib.load_manifest('atlas_teleop')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Int8

import math

class DrcVehicleTeleop:

    def __init__(self):
        rospy.init_node('drc_vehicle_teleop')
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.robot_enter_car = rospy.Publisher('drc_world/robot_enter_car', Pose)
        self.robot_exit_car = rospy.Publisher('drc_world/robot_exit_car', Pose)
        self.brake_pedal = rospy.Publisher('drc_vehicle/brake_pedal/cmd', Float64)
        self.gas_pedal = rospy.Publisher('drc_vehicle/gas_pedal/cmd', Float64)
        self.hand_brake = rospy.Publisher('drc_vehicle/hand_brake/cmd', Float64)
        self.hand_wheel = rospy.Publisher('drc_vehicle/hand_wheel/cmd', Float64)
        self.direction = rospy.Publisher('drc_vehicle/direction/cmd', Int8)
        
        self.AXIS_HAND_BRAKE = 0
        self.AXIS_BRAKE_PEDAL = 1
        self.AXIS_DIRECTION = 2
        self.AXIS_GAS_PEDAL = 3
        self.AXIS_HAND_WHEEL = 4

        self.BUTTON_ENTER_CAR = 0
        self.BUTTON_EXIT_CAR = 1

    def joy_cb(self, data):
        if data.buttons[self.BUTTON_ENTER_CAR] == 1:
            self.robot_enter_car.publish(Pose())
        elif data.buttons[self.BUTTON_EXIT_CAR] == 1:
            self.robot_exit_car.publish(Pose())
        else:
            self.hand_brake.publish(Float64(data.axes[self.AXIS_HAND_BRAKE]))
            self.brake_pedal.publish(Float64(data.axes[self.AXIS_BRAKE_PEDAL]))
            self.gas_pedal.publish(Float64(data.axes[self.AXIS_GAS_PEDAL]))
            direction = -1 if data.axes[self.AXIS_DIRECTION] < 0.5 else 1
            self.direction.publish(Int8(direction))
            hand_wheel = (data.axes[self.AXIS_HAND_WHEEL] - 0.5) * math.pi
            self.hand_wheel.publish(Float64(hand_wheel))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    d = DrcVehicleTeleop()
    d.run()

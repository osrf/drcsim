#! /usr/bin/env python
import roslib; roslib.load_manifest('atlas_utils')

from atlas_msgs.msg import WalkDemoAction, \
                           WalkDemoActionGoal, \
                           WalkDemoGoal, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorStepParams, \
                           AtlasBehaviorStandParams, \
                           AtlasBehaviorManipulateParams
from std_msgs.msg import Header

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import actionlib
import math
import rospy
import select
import sys
import termios
import tty

class AtlasTeleop():
    
    # Keyboard teleop bindings
    directions = {'u': {"forward":1, "lateral":0, "turn": 1}, \
                  'U': {"forward":1, "lateral":0, "turn": 1}, \
                  'i': {"forward":1, "lateral":0, "turn": 0}, \
                  'I': {"forward":1, "lateral":0, "turn": 0}, \
                  'o': {"forward":1, "lateral":0, "turn":-1}, \
                  'O': {"forward":1, "lateral":0, "turn":-1}, \
                  'j': {"forward":0, "lateral":0.25, "turn": 0}, \
                  'J': {"forward":0, "lateral":0.25, "turn": 0}, \
                  'k': {"forward":0, "lateral":0, "turn": 0}, \
                  'K': {"forward":-1, "lateral":0, "turn": 0}, \
                  'l': {"forward":0, "lateral":-0.25, "turn": 0}, \
                  'L': {"forward":0, "lateral":-0.25, "turn": 0}, \
                  'm': {"forward":0, "lateral":0, "turn": 0.5}, \
                  ',': {"forward":-0.5, "lateral":0, "turn": 0}, \
                  '.': {"forward":0, "lateral":0, "turn":-0.5}}
    
    # BDI Controller bindings
    params = {"stride_length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "step_height":{"value":0, "min":-1, "max":1, "type":"float"},
              "stride_duration":{ "value":0.63, "min": 0, "max":100, \
                                  "type":"float"},
              "sequence_length":{"value":5, "min":1, "max":100, "type":"int"},
              "stride_width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "in_place_turn_size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "turn_radius":{"value":2, "min":0.01, "max":100, "type":"float"},
              "swing_height":{"value":0.3, "min":0, "max":1, "type":"float"}}
    
    def init(self):
        # Saves terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Creates the SimpleActionClient, passing the type of the action
        # () to the constructor.
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', \
          WalkDemoAction)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
          String, None, False, True, None)
    
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("Waiting for atlas/bdi_control")
        self.client.wait_for_server()

    def fini(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def run(self):
        try:
            self.init()
            self.print_usage()
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

    def print_usage(self):
        msg = """
        Keyboard Teleop for AtlasSimInterface 1.0.5
        Copyright (C) 2013 Open Source Robotics Foundation
        Released under the Apache 2 License
        --------------------------------------------------
        Linear movement:

                i    
           j         l
                ,    
                
        Turn movements:
        o/u Turn around a point
        m/. Turn in place
        
        1-9: Change the length of step trajectory
        E: View and Edit Parameters
        R: Reset robot to standing pose
        Q: Quit
        """
        self.loginfo(msg)      
        
    # Publishes commands to reset robot to a standing position
    def reset_to_standing(self):
        self.mode.publish("harnessed")
        self.control_mode.publish("Freeze")
        self.control_mode.publish("StandPrep")
        rospy.sleep(2.0)
        self.mode.publish("nominal")
        rospy.sleep(0.3)
        self.control_mode.publish("Stand")

    # Builds a trajectory of step commands. 
    # Param forward: 1 forward, -1 backward or 0 if no forward component
    # Param lateral: 1 left, -1 right, 0 if no lateral component
    # Param turn: 1 Counter clockwise turn, -1 clockwise turn    
    def twist(self, forward, lateral, turn):
        self.loginfo("Walking " + \
        str(self.params["sequence_length"]["value"]) + " steps")
        steps = []
        
        L = self.params["stride_length"]["value"]
        R = self.params["turn_radius"]["value"]
        W = self.params["stride_width"]["value"]
        X = 0
        Y = 0
        theta = 0
        dTheta = 0
        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + \
            self.params["stride_width"]["value"]/2)))
        else:
            dTheta = turn * self.params["in_place_turn_size"]["value"]
        steps = []
        
        # This home step doesn't currently do anything, but it's a 
        # response to bdi not visiting the first step in a trajectory
        home_step = AtlasBehaviorStepData()
        home_step.foot_index = 1
        home_step.pose.position.y = 0.1
        steps.append(home_step)
        prevX = 0
        prevY = 0
        
        # Builds the sequence of steps needed
        for i in range(self.params["sequence_length"]["value"]):
            
            theta += (turn != 0) * dTheta
            # left = 1, right = -1
            foot = 1 - 2 * (i % 2)
            
            
            if turn == 0:
                X = (forward != 0) * (X + forward * L)
                Y = (lateral != 0) * (Y + lateral * L) - foot * W / 2
            elif forward != 0:
                self.debuginfo("R: " + str(R) + " R_foot:" + \
                str(R_foot) + " theta: " + str(theta) +  \
               " math.sin(theta): " + str(math.sin(theta)) + \
               " math.cos(theta) + " + str(math.cos(theta)))
                
                # Radius from point to foot (if turning)
                R_foot = R + foot * W/2
                
                # turn > 0 for CCW, turn < 0 for CW
                X = forward * turn * R_foot * math.sin(theta)
                Y = forward * turn * (R - R_foot*math.cos(theta))
            elif turn != 0:
                X = turn * W/2 * math.sin(theta)
                Y = turn * W/2 * math.cos(theta)
            
            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()
            
            # One step already exists, so add one to index
            step.step_index = i+1
            
            # Alternate between feet, start with left
            step.foot_index = i % 2
            step.duration = self.params["stride_duration"]["value"]
            
            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = self.params["step_height"]["value"]
         
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            
            step.swing_height = self.params["swing_height"]["value"]

            self.debuginfo("foot: " + str(step.foot_index) + " [" + \
              str(step.pose.position.x) + ", " + \
              str(step.pose.position.y) + ", " + str(theta) + "]")            
            steps.append(step)
        
        # Add final step to bring feet together
        # foot = 1 for left, foot = -1 for right
        foot = 1 - 2 * (1 - steps[-1].foot_index)
        
        
        if turn == 0:
            Y = (lateral != 0) * (Y + lateral * L/2) - foot * W / 2
        elif forward != 0:
            self.debuginfo("R: " + str(R) + " R_foot:" + \
            str(R_foot) + " theta: " + str(theta) +  \
           " math.sin(theta): " + str(math.sin(theta)) + \
           " math.cos(theta) + " + str(math.cos(theta)))
            
            # R_foot is radius to foot
            R_foot = R + foot * W/2
            #turn > 0 for counter clockwise
            X = forward * turn * R_foot * math.sin(theta)
            Y = forward * turn * (R - R_foot*math.cos(theta))
        else:
            X = turn * W/2 * math.sin(theta)
            Y = turn * W/2 * math.cos(theta)
            
        Q = quaternion_from_euler(0, 0, theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = 1 - steps[-1].foot_index
        step.duration = self.params["stride_duration"]["value"]
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = self.params["step_height"]["value"]
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["swing_height"]["value"]
        self.debuginfo("foot: " + str(step.foot_index) + \
          " [" + str(step.pose.position.x) + \
          ", " + str(step.pose.position.y) + ", " + str(theta) + "]")               
        steps.append(step)

        # 0 for full BDI control, 255 for PID control
        k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
               
        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps, \
          AtlasBehaviorStepParams(), AtlasBehaviorStandParams(), \
          AtlasBehaviorManipulateParams(),  k_effort )
        
        self.client.send_goal(walk_goal)

    # Select binding values and call twist
    def process_movement(self, ch):
        dir = self.directions[ch]       
        self.twist(dir["forward"], dir["lateral"], dir["turn"])
    
    # Puts teleop into edit param mode
    def edit_params(self):
        # Reset terminal to normal settings so you can see input
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("")
        
        # Find the longest param, and use that to justify strings
        maxLength = -1
        for key in self.params.keys():
            if len(key) > maxLength:
                maxLength = len(key)
        for i in range(len(self.params)):
            param_name = self.params.keys()[i]
            print(str(i) + " : " + param_name.ljust(maxLength + 1) + \
              str(self.params[param_name]["value"]))
        print("X : Exit")
        hasNumber = False
        selection = -1
        
        # Get the input, and check if it's valid
        while selection < 0 or selection >= len(self.params):
            var = raw_input("Enter number of param you want to change: ")
            
            if var == 'x' or var == 'X':
                self.print_usage()
                return
            try:
                selection = int(var)
            except ValueError:
                selection = -1
        
        param = self.params.keys()[selection]
        value = 0
        valid = False
        
        # Keep asking for input while the value is not valid, or it is outside 
        # of the acceptable range
        while not valid:
            var = raw_input("New value for " + param + " [min: " +
            str(self.params[param]["min"]) + ", max: " +
            str(self.params[param]["max"]) + ", type: " +
            str(self.params[param]["type"]) + "]? ") 
            try:
                if (self.params[param]["type"] is "float"):
                    value = float(var)
                elif (self.params[param]["type"] is "int"):
                    value = int(var)
                valid = (value >= self.params[param]["min"] and \
                         value <= self.params[param]["max"])
            except ValueError:
                valid = False
        
        self.params[param]["value"] = value
        self.edit_params()
        
    # Used to print items to screen, while terminal is in funky mode
    def loginfo(self, str):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.loginfo(str)
        tty.setraw(sys.stdin.fileno())
    
    # Used to print debug items to screen while terminal is funky
    def debuginfo(self, str):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.logdebug(str)
        tty.setraw(sys.stdin.fileno())
    
    # For everything that can't be a binding, use if/elif instead
    def process_key(self, ch):
        if self.directions.has_key(ch):
            self.process_movement(ch)
        elif ch == 'e' or ch == 'E':
            self.edit_params()
        elif ch == 'r':
            self.reset_to_standing()
        elif ch == 'h' or ch == 'H':
            self.print_usage()
        elif ch == 'q' or ch == 'Q':
            rospy.signal_shutdown("Shutdown")
        try:
            if (int(ch) >= self.params["sequence_length"]["min"] and \
                int(ch) <= self.params["sequence_length"]["max"]):
                self.params["sequence_length"]["value"] = int(ch)
                self.loginfo("sequence_length: " + \
                  str(self.params["sequence_length"]["value"]))
        except ValueError:
            pass
    
    # Get input from the terminal
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key
 
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()

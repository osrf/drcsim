#! /usr/bin/env python

import roslib; roslib.load_manifest('AtlasActionClient')
import rospy

# Brings in the SimpleActionClient
import actionlib, math, select, sys, termios, tty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from atlas_msgs.msg import AtlasSimInterface, \
                           AtlasSimInterfaceGoal, \
                           AtlasSimInterfaceAction, \
                           AtlasBehaviorStepParams
                           
from tf.transformations import quaternion_from_euler, euler_from_quaternion



class AtlasTeleop():
    
    directions = {'u': {"forward":1, "lateral":0, "turn": 1}, \
                  'U': {"forward":1, "lateral":0, "turn": 1}, \
                  'i': {"forward":1, "lateral":0, "turn": 0}, \
                  'I': {"forward":1, "lateral":0, "turn": 0}, \
                  'o': {"forward":1, "lateral":0, "turn": -1}, \
                  'O': {"forward":1, "lateral":0, "turn": -1}, \
                  'j': {"forward":0, "lateral":0.25, "turn": 0}, \
                  'J': {"forward":0, "lateral":0.25, "turn": 0}, \
                  'k': {"forward":0, "lateral":0, "turn": 0}, \
                  'K': {"forward":-1, "lateral":0, "turn": 0}, \
                  'l': {"forward":0, "lateral":-0.25, "turn": 0}, \
                  'L': {"forward":0, "lateral":-0.25, "turn": 0}, \
                  'm': {"forward":0, "lateral":0, "turn": 0.5}, \
                  ',': {"forward":-0.5, "lateral":0, "turn": 0}, \
                  '.': {"forward":0, "lateral":0, "turn": -0.5}}
    
    params = {"stride_length":{ "value":0.25, "min":0, "max":1, "type":"float"},
              "stride_duration":{ "value":0.63, "min": 0, "max":100, "type":"float"},
              "sequence_length":{"value":5, "min":2, "max":100, "type":"int"},
              "stride_width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "in_place_turn_size":{"value":math.pi/32, "min":0.01, "max":math.pi/2, "type":"float"},
              "turn_radius":{"value":1, "min":0.01, "max":100, "type":"float"},
              "swing_height":{"value":0.3, "min":0, "max":1, "type":"float"}}
    
    def init(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Creates the SimpleActionClient, passing the type of the action
        # () to the constructor.
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', AtlasSimInterfaceAction)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', String, None, False, True, None)
    
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("Waiting for atlas/bdi_control server")
        self.client.wait_for_server()

    def fini(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def run(self):
        try:
            self.init()
            self.printUsage()
            while not rospy.is_shutdown():
                ch =  self.getKey()
                self.processKey(ch)
        finally:
            self.fini()

    def printUsage(self):
        msg = """
        Keyboard Teleop for AtlasSimInterface 1.0.5
        Copyright (C) 2013 Open Source Robotics Foundation
        Released under the Apache 2 License
        --------------------------------------------------
        Movement:

           u    i    o
           j    k    l
           m    ,    .
        
        2-9: Change the length of step trajectory
        E: View and Edit Parameters
        R: Reset robot to standing pose
        Q: Quit
        """
        self.debug(msg)      
        
    def resetToStanding(self):
        self.mode.publish("harnessed")
        self.control_mode.publish("stand-prep")
        rospy.sleep(5.0)
        self.mode.publish("nominal")
        rospy.sleep(0.3)
        self.control_mode.publish("stand")
    
    def stand(self):
        self.control_mode.publish("stand")
        rospy.sleep(1)
    
    def twist(self, forward, lateral, turn):
        self.debug("Walking " + str(self.params["sequence_length"]["value"]) + " steps")
        steps = []
        
        L = self.params["stride_length"]["value"]
        R = self.params["turn_radius"]["value"]
        W = self.params["stride_width"]["value"]
        X = 0
        Y = 0
        theta = 0
        dTheta = 0
        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L/(2*R))
        else:
            dTheta = turn * self.params["in_place_turn_size"]["value"]
        steps = []
        for i in range(self.params["sequence_length"]["value"]):
            theta += (turn!=0) * (i%2) * dTheta
            #left = 1, right = -1
            foot = 1 - 2*(i%2)
            
            if turn == 0:
                X = (forward!=0) * (X + forward * L)
                Y = (lateral!=0) * (Y + lateral * L) - foot * W/2
            else:
                X = forward * turn * R * math.sin(theta) - foot * W/2 * math.sin(theta)
                Y = forward * turn * (R - R * math.cos(theta)) - foot * W/2 * math.cos(theta)
            
            Q = quaternion_from_euler(0,0,theta)

            step = AtlasBehaviorStepParams()
            step.step_index = i+1
            step.foot_index = i % 2
            step.duration = self.params["stride_duration"]["value"]
            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = 0.1
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            step.swing_height = self.params["swing_height"]["value"]
            steps.append(step)
        
        #Add final step to bring feet together
        #left = 1, right = -1
        foot = 1 - 2 * (1 - steps[-1].foot_index)
        X = X - foot * W/2 * math.sin(theta)
        Y = Y - foot * W/2 * math.cos(theta)
        Q = quaternion_from_euler(0,0,theta)
        step = AtlasBehaviorStepParams()
        step.step_index = len(steps) + 1
        step.foot_index = 1 - steps[-1].foot_index
        step.duration = self.params["stride_duration"]["value"]
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = 0.1
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["swing_height"]["value"]
        steps.append(step)
               
           
        multi_step_walk_goal = AtlasSimInterfaceGoal(AtlasSimInterface(None, 0, rospy.Time.now() , AtlasSimInterface.MULTI_STEP_WALK, steps, None, None))
        
        self.client.send_goal(multi_step_walk_goal)
        self.client.wait_for_result(rospy.Duration(2*self.params["stride_duration"]["value"]*len(steps)))
    
    def processMovement(self, ch):
        self.newMovement = True
        self.stand()
#        self.client.wait_for_result()

        dir = self.directions[ch]       
        self.twist(dir["forward"], dir["lateral"], dir["turn"])
    
    def editParams(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("")
        maxLength = -1
        for key in self.params.keys():
            if len(key) > maxLength:
                maxLength = len(key)
        for i in range(len(self.params)):
            param_name = self.params.keys()[i]
            print(str(i) + " : " + param_name.ljust(maxLength+1) + str(self.params[param_name]["value"]))
        print("X : Exit")
        hasNumber = False
        selection = -1
        while selection < 0 or selection >= len(self.params):
            var = raw_input("Enter number of param you want to change: ")
            if var == 'x' or var == 'X':
                self.printUsage()
                return
            try:
                selection = int(var)
            except ValueError:
                selection = -1
        
        param = self.params.keys()[selection]
        value = 0
        valid = False
        while not valid:
            var = raw_input("New value for " + param + " [min: " + str(self.params[param]["min"]) + ", max: " + str(self.params[param]["max"]) + ", type: " + str(self.params[param]["type"]) + "]? ")
            try:
                if (self.params[param]["type"] is "float"):
                    value = float(var)
                elif (self.params[param]["type"] is "int"):
                    value = int(var)
                valid = (value >= self.params[param]["min"] and value <= self.params[param]["max"])
            except ValueError:
                valid = False
        
        self.params[param]["value"] = value
        self.editParams()
        
    def debug(self, str):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print(str)
        tty.setraw(sys.stdin.fileno())
    
    def processKey(self, ch):
        if self.directions.has_key(ch):
            self.processMovement(ch)
        elif ch == 'e' or ch == 'E':
            self.editParams()
        elif ch == 'r':
            self.resetToStanding()
        elif ch == 'h' or ch == 'H':
            self.printUsage()
        elif ch == 'q' or ch == 'Q':
            rospy.signal_shutdown("Shutdown")
        try:
            if (int(ch) > 1):
                self.params["sequence_length"]["value"] = int(ch)
                self.debug("sequence_length: " + str(self.params["sequence_length"]["value"]))
        except ValueError:
            pass
            
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key
 
    
if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()
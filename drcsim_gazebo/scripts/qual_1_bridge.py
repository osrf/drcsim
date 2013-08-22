#! /usr/bin/env python
import roslib; roslib.load_manifest('vrc_cheats')

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
    
    def init(self):

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

    # Script to build steps to walk across qual_world_1 bridge
    # Atlas needs to start at the edge of the main platform, just slightly to 
    # the right-hand side of the yellow line.  
    def bridge_qual_1(self):
        s = []
        # Step to first block
        s.append([0.5, 0, 0])
        # Inch forward on first block
        for i in range(3):
            s.append([0.03, 0, 0])
        # Turn toward second block
        for i in range(4):
            s.append([0, 0, 0.196])
        # Step onto second block
        s.append([0.4, 0, 0])
        s.append([0.4, 0, 0])

        # Turn toward third block
        for i in range(4):
            s.append([0, 0, -0.196])
            
        # Inch forward on second block
        for i in range(3):
            s.append([0.03, 0, 0])
            
        # Step to third block
        s.append([0.4, 0, 0])
        #Inch forward on third block
        for i in range(4):
            s.append([0.03, -0.01, 0])
            
        # Turn toward fourth block
        for i in range(4):
            s.append([0, 0, -0.196])
            
        #Step onto fourth block
        s.append([0.4, 0, 0])
        s.append([0.4, 0, 0])
        
        # Turn toward goal 
        for i in range(4):
            s.append([0, 0, 0.196])    
        
        #Step onto platform
        s.append([0.4, 0, 0])
        s.append([0.4, 0, 0])        
        #Walk through the goal
        for i in range(10):
            s.append([0.3, 0, 0])

        steps = []
            
        home_step = AtlasBehaviorStepData()
        
        # If moving right, first dummy step is on the left
        home_step.foot_index = 1
        home_step.pose.position.y = -0.1
        steps.append(home_step)
        X = 0
        Y = 0
        W = 0.2
        theta = 0
        
        # Build trajectory steps
        for i in range(len(s)):
            is_even = i%2
            is_odd = 1-is_even
            is_right_foot = is_even
            foot = 1 - 2*is_even
            turn = 0
           
            traj = s[i]
            theta += traj[2]
            if traj[2] > 0:
                turn = 1
            elif traj[2] < 0:
                turn = -1 
            X += traj[0]*math.sin(3.14/2 - theta) + traj[1]*math.sin(theta) 
            Y += traj[0]*math.cos(3.14/2 - theta) + traj[1]*math.cos(theta)
            print("[" + str(X) + ", " + str(Y) + ", " + str(theta))
            
            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()
            
            # One step already exists, so add one to index
            step.step_index = i+1
            
            # Alternate between feet, start with left
            step.foot_index = is_right_foot
                       
            step.duration = 0.63
            
            step.pose.position.x = X - foot * W/2 * math.sin(theta)
            step.pose.position.y = Y + foot * W/2 * math.cos(theta)
            step.pose.position.z = 0
         
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            
            step.swing_height = 0.3
            steps.append(step)
            
        k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
               
        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps, \
          AtlasBehaviorStepParams(), AtlasBehaviorStandParams(), \
          AtlasBehaviorManipulateParams(),  k_effort )
        
        self.client.send_goal(walk_goal)
        
        for step in steps:
            print("foot: " + str(step.foot_index) + \
              " [" + str(step.pose.position.x) + \
              ", " + str(step.pose.position.y) + ", " + str(theta) + "]")   
 
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.init()
    teleop.bridge_qual_1()

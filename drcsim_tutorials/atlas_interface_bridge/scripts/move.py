#!/usr/bin/env python
#
# Copyright 2013 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import roslib
roslib.load_manifest('atlas_interface_bridge')

import rospy
import time
import math
from atlas_msgs.msg import AtlasCommand, AtlasState

NUM_JOINTS = 28

def go():
    pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)
    while not rospy.is_shutdown():
      ac = AtlasCommand()
      # Wiggle it, just a little bit
      pos = 0.1 * math.sin(time.time())
      for i in range(0, NUM_JOINTS):
          ac.position.append(pos)
          ac.velocity.append(0.0)
          ac.effort.append(0.0)
      pub.publish(ac)
      time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('move', anonymous=True)
    go()

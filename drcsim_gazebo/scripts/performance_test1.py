#!/usr/bin/env python
#
# Copyright 2012 Open Source Robotics Foundation
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
#
#
# this programs takes BDI's cfg file, and generates an URDF */

PKG = 'drcsim_gazebo'
NAME = 'performance_test1'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from rosgraph_msgs.msg import Clock

class PerformanceTest(unittest.TestCase):
    def __init__(self, *args):
        super(PerformanceTest, self).__init__(*args)
        self.success = False
        self.max_factor_reached = 0
        self.factor_required = 0.50

        # test duration in real time seconds
        self.test_duration = 10.0

        # test start time in sim time seconds
        self.test_start_time = 12.0

        self.clock_topic = "/clock"
        self.got_clock_t0 = False
        self.clock_t0 = Clock.clock
        self.clock_t1 = Clock.clock
        self.wallclock_t0 = rospy.Time()
        self.wallclock_t1 = rospy.Time()

    def test_performance(self):
        print "LNK\n"
        rospy.init_node(NAME, anonymous=True)

        self.test_start_time = rospy.get_param("test_start_sim_time",self.test_start_time);
        self.test_duration = rospy.get_param("test_duration",self.test_duration);

        while not rospy.is_shutdown() and rospy.Time.now() < rospy.Time(self.test_start_time):
            rospy.loginfo("Waiting for test to start at time [%s]"% self.test_start_time)
            time.sleep(0.1)

        start_t_real = time.time()
        start_t_sim = rospy.Time.now()

        timeout_t = start_t_real + self.test_duration
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            print "elasped sim ", rospy.Time.now().to_sec() - start_t_sim.to_sec()
            print "elasped real ", time.time() - start_t_real
            factor = (rospy.Time.now().to_sec() - start_t_sim.to_sec()) / (time.time() - start_t_real)
            print "real time factor ", factor
            if factor > self.max_factor_reached:
                self.max_factor_reached = factor
            if factor > self.factor_required:
                self.success = True
            
            time.sleep(0.1)


        self.assertGreaterEqual(self.max_factor_reached, self.factor_required, "Performance factor desired (" + str(self.factor_required) + ") was not reached: " + str(self.max_factor_reached))
        
if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, sys.argv[0], PerformanceTest, sys.argv) #, text_mode=True)




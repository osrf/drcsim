#!/usr/bin/env python

from __future__ import print_function
import unittest
import rostest
import subprocess
import sys
import time
import re
import rospy

class TestStopLog(unittest.TestCase):
    
    def wait_gazebo_to_start(self):
        # Wait until /clock is being published; this can take an unpredictable
        # amount of time when we're downloading models.
        while rospy.Time.now().to_sec() == 0.0:
            print('Waiting for Gazebo to start...')
            time.sleep(1.0)
        # Take an extra nap, to allow plugins to be loaded
        time.sleep(5.0)

    def wait_until_gazebo_unpaused(self):
        not_finished = True
        while (not_finished):
            p1 = subprocess.Popen(["timeout", "1", "gzstats", "-p"], \
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            p2 = subprocess.Popen(["cut", "-d", "," , "-f", "4" ], \
                    stdin=p1.stdout, stdout=subprocess.PIPE)
            p3 = subprocess.Popen(["tail", "-n", "1"], \
                    stdin=p2.stdout, stdout=subprocess.PIPE)
            output = p3.communicate()[0]
            if output.strip() == "F":
                not_finished = False

    def run_gzstop(self):
        cmd = ['gzlog', 'stop']
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, \
                                   stderr=subprocess.PIPE)
        out, err = po.communicate()
        self.assertEqual(po.returncode, 0, \
          'gzlog stop failed (%s). stdout: %s stderr: %s'%(cmd, out, err))

    def kill_gzserver(self):
        cmd = ['killall', '-INT', 'gzserver']
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, \
                                   stderr=subprocess.PIPE)
        out, err = po.communicate()
        self.assertEqual(po.returncode, 0, \
          'kill roslaunch failed (%s). stdout: %s stderr: %s'%(cmd, out, err))

    def check_closing_log(self,logfile):
        try:
            file = open(logfile, "r")
        except IOError as e:
            self.fail("Can not open log file error({0}): {1}". \
                    format(e.errno, e.strerror))

        found = False
        for line in file:
            # if found, It should be the last line, never be here.
            # This check will also cover the existance of two tags
            if found:
                file.close()
                self.fail("Not the final line for closing tag")

            if re.search('^</gazebo_log>',line):
                found = True

        file.close()
        return found

    def test_gzlog(self):
        try:
            time_to_publish = float(rospy.get_param('/gzstop_checker/time_to_publish'))
            logfilename = rospy.get_param('/gzstop_checker/logfile')
        except KeyError, e:
            self.fail('gzlog test not initialized properly. Parameter [%s] not set. debug[%s] debug[%s]'%(str(e), \
                      rospy.get_caller_id(), rospy.resolve_name(e.args[0])))

        self.wait_gazebo_to_start()
        time.sleep(time_to_publish)
        self.run_gzstop()
        time.sleep(2)
        # Need to wait until gazebo back from paused to give time to write log
        self.wait_until_gazebo_unpaused()
        #self.kill_gzserver()
        self.assertTrue(self.check_closing_log(logfilename), "Can not find closing tag in log file")
         
if __name__ == '__main__':
    print('Sleeping to let Gazebo start...')
    rospy.init_node('gzstop_checker_node', anonymous=True)
    rostest.run('atlas_utils', 'gzlog', TestStopLog)

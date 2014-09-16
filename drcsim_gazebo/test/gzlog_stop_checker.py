#!/usr/bin/env python

from __future__ import print_function
import unittest
import rostest
import subprocess
import sys
import time
import re
import rospy
import os

class TestStopLog(unittest.TestCase):
    
    def wait_gazebo_to_start(self):
        # Wait until /clock is being published; this can take an unpredictable
        # amount of time when we're downloading models.
        while rospy.Time.now().to_sec() == 0.0:
            print('Waiting for Gazebo to start...')
            time.sleep(1.0)
        # Take an extra nap, to allow plugins to be loaded
        time.sleep(5.0)

    def load_params(self):
        try:
            time_to_publish = \
               float(rospy.get_param('/gzstop_checker/time_to_publish'))
            logfilename = \
               rospy.get_param('/gzstop_checker/logfile')
            results_postfix = \
               rospy.get_param('/gzstop_checker/results_postfix')
        except KeyError, e:
            self.fail('gzlog test not initialized properly. Parameter [%s] not set. debug[%s] debug[%s]'%(str(e), \
                      rospy.get_caller_id(), rospy.resolve_name(e.args[0])))

        return time_to_publish, logfilename, results_postfix

    def get_gzserver_pid(self):
        p1 = subprocess.Popen(["pidof", "gzserver"], stdout=subprocess.PIPE)
        return p1.communicate()[0].strip()

    def get_gzserver_mem(self):
        gzserver_pid = self.get_gzserver_pid()

        if (not gzserver_pid):
            return 0;

        p1 = subprocess.Popen(["cat", "/proc/" + gzserver_pid + "/status"], \
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        p2 = subprocess.Popen(["grep", "VmSize"], \
                stdin=p1.stdout, stdout=subprocess.PIPE)
        p3 = subprocess.Popen(["awk", "{ print $2 }"], \
                stdin=p2.stdout, stdout=subprocess.PIPE)

        current_mem = float(p3.communicate()[0])

        if current_mem > self.max_mem_consumed:
            self.max_mem_consumed = current_mem

    def write_memory_stats(self, results_postfix):
        try:
            logdir = os.environ['ROS_TEST_RESULTS_DIR']
            f = open(logdir + '/' + results_postfix + '_max_mem_used.txt','w')
            f.write(str(self.max_mem_consumed))
            f.close()
            print ("Max memory consumed by gzserver: " + str(self.max_mem_consumed))
        except IOError as e:
            print ("Can not write mam_mem_used file({0}): {1}". \
                    format(e.errno, e.strerror))

    def write_time_stats(self, duration, results_postfix):
        try:
            logdir = os.environ['ROS_TEST_RESULTS_DIR']
            f = open(logdir + '/' + results_postfix + '_time_used.txt','w')
            f.write(str(duration))
            f.close()
            print ("Duration of waiting for log: " + str(duration))
        except IOError as e:
            print ("Can not write time_used file({0}): {1}". \
                  format(e.errno, e.strerror))

    def wait_until_gazebo_unpaused(self):
        not_finished = True
        start = time.time()
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
            # logging
            self.get_gzserver_mem()
        end = time.time()
        return (end - start)

    def run_gzstop(self):
        cmd = ['gzlog', 'stop']
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, \
                                   stderr=subprocess.PIPE)
        out, err = po.communicate()
        self.assertEqual(po.returncode, 0, \
          'gzlog stop failed (%s). stdout: %s stderr: %s'%(cmd, out, err))

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
        self.max_mem_consumed = 0

        # Wait to gazebo to start the testing
        self.wait_gazebo_to_start()
        # Load from parameter server
        time_to_publish, logfilename, results_postfix = self.load_params()
        # Wait all testing time
        time.sleep(time_to_publish)
        # Launch the gzstop signal
        self.run_gzstop()
        time.sleep(2)
        # Wait until gazebo back from paused to give time to write log
        duration = self.wait_until_gazebo_unpaused()
        self.write_time_stats(duration, results_postfix)
        self.write_memory_stats(results_postfix)

        self.assertTrue(self.check_closing_log(logfilename), "Can not find closing tag in log file")
         
if __name__ == '__main__':
    rospy.init_node('gzstop_checker_node', anonymous=True)
    rostest.run('drcsim_gazebo', 'gzlog', TestStopLog)

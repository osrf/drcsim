#!/usr/bin/env python

from __future__ import print_function
import unittest
import subprocess
import sys

class Tester(unittest.TestCase):

    def setUp(self):
        self.argv = sys.argv
        self.skip = False
        try:
            import yaml
        except:
            print('WARNING: Failed to import yaml, so skipping ROS API checks.  On Ubuntu: sudo apt-get install python-yaml')
            self.skip = True

    def test_api(self):
        self.assertTrue(len(self.argv) > 2, msg='Not enough args')
        topics = self._load(self.argv[1:])
    
    def _test_topic(t):
        cmd = ['rostopic', 'info', t.topic]
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = po.communicate()
        #if po.returncode:
        
    
    def _load(files):
        topics = []
        files = argv[1:]
        for f in files:
            # Let parsing exceptions leak out; they'll be marked as test errors
            y = yaml.load(open(f))
            self.assertIn('topics', y)
            self.assertIn('topic', y['topics'])
            self.assertIn('topic', y['type'])
            self.assertIn('topic', y['num_publishers'])
            self.assertIn('topic', y['num_subscribers'])
            topics.extend(y['topics'])
        return topics

if __name__ == '__main__':
    unittest.main()

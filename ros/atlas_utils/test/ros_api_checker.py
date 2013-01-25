#!/usr/bin/env python

from __future__ import print_function
import unittest
import subprocess
import sys

try:
    import yaml
except:
    print('WARNING: Failed to import yaml, so skipping ROS API checks.  On Ubuntu: sudo apt-get install python-yaml')
    # TODO: synthesize skipped test result file
    sys.exit(1)

def go(argv):
    if len(argv) < 2:
        print('Not enough args')
        #TODO: fail
        sys.exit(1)
    topics = load(argv[1:])

def test_topic(t):
    cmd = ['rostopic', 'info', t.topic]
    po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = po.communicate()
    if po.returncode:
    

def load(files):
    topics = []
    files = argv[1:]
    for f in files:
        try:
            y = yaml.load(open(f))        
            ts = y['topics']
            for t in ts:
                # reference all fields to cause exception
                s = t['topic']
                s = t['type']
                s = t['num_publishers']
                s = t['num_subscribers']
            topics.extend(ts)
        except Exception as e:
            print('Parsing %s failed with: %s'%(f,`e`))
            #TODO: fail
            sys.exit(1)
    return topics

if __name__ == '__main__':
    go(sys.argv)

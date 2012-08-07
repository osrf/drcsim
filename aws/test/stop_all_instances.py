#!/usr/bin/env python

from __future__ import print_function
import sys
import boto

# No args: uses config from ~/.boto
ec2 = boto.connect_ec2()

reservations = ec2.get_all_instances()
for r in reservations:
    instances = r.instances
    for i in instances:
        if i.state == 'running':
            print('Trying to stop instance %s...'%(i.id), end='')
            sys.stdout.flush()
            try:
                started = ec2.stop_instances([i.id])
                print('Succeeded.')
            except Exception as e:
                print('Failed with exception: %s'%(e))
        else:
            print('Skipping instance %s because its state is \'%s\''%(i.id, i.state))

#!/usr/bin/env python

import boto

# No args: uses config from ~/.boto
ec2 = boto.connect_ec2()

reservations = ec2.get_all_instances()
for r in reservations:
  instances = r.instances
  for i in instances:
      print 'Instance %s at IP address %s is in state \'%s\'.'%(i.id, i.ip_address if i.ip_address else '(none)', i.state)

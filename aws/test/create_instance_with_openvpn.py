#!/usr/bin/env python

from __future__ import print_function
import sys
import uuid
import boto
import os

# The image to use; 'ami-443b474' is 64-bit Ubuntu 12.04.
IMAGE_ID = 'ami-4438b474'
# Instance type.  't1.micro' is the freebie.
INSTANCE_TYPE = 't1.micro'
# Security groups.  'openvpn' is configured to allow ssh and openvpn
SECURITY_GROUPS = ['openvpn']
# Startup script
STARTUP_SCRIPT = """#!/bin/bash
apt-get update
apt-get install -y openvpn
openvpn --genkey --secret static.key
echo <<DELIM > openvpn.config
dev tun
ifconfig 10.8.0.1 10.8.0.2
secret static.key
DELIM
sudo openvpn --config openvpn.config
"""
# Where to dump config files
CONFIG_DIR = os.getcwd()

def go():
    # No args: uses config from ~/.boto
    ec2 = boto.connect_ec2()

    # Create key pair to use for SSH access.  Note that 
    # create_key_pair() registers the named key with AWS.
    uid = uuid.uuid1()
    kp_name = 'key-%s'%(uid)
    kp = ec2.create_key_pair(kp_name)

    try:
        res = ec2.run_instances(image_id=IMAGE_ID, key_name=kp_name, instance_type=INSTANCE_TYPE, security_groups=SECURITY_GROUPS)
        kp.save(CONFIG_DIR)
        #TODO:
        # retrieve the static key
        # create openvpn config file
        # print stuff out
    except Exception as e:
        kp.delete()
        raise e

if __name__ == '__main__':
    go()

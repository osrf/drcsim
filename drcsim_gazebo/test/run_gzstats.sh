#!/bin/bash

# Wait for all the other nodes to hook up
sleep 10

gzstats -p > gzstats.log

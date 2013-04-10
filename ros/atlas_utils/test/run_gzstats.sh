#!/bin/bash

# Wait for all the other nodes to hook up
sleep 10

TIMESTAMP=$(date +"%Y-%m-%dT%H:%M:%S")
LOG_DIR="${HOME}/.ros/"
LOG_FILE="${LOG_DIR}/gzstats.log"

mkdir -p ${LOG_DIR}
gzstats -p > "${LOG_FILE}"

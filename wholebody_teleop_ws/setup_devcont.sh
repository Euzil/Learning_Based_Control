#!/bin/bash

if ! (return 0 2>/dev/null) ; then
    >&2 echo "ERROR: You must source this script."
    exit 1
fi

echo "Sourcing setup_devcont.sh ..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=69

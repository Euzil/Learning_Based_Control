#!/bin/bash

if [ -d /scripts/BashAddons ]; then
	source /scripts/BashAddons/init.sh
fi

ross()
{
    . /opt/ros/$ROS_DISTRO/setup.bash
}

rosmaster_local()
{
    export ROS_DOMAIN_ID=69
    export ROS_LOCALHOST_ONLY=1
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

rosmaster_remote()
{
    export ROS_DOMAIN_ID=69
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

if [[ "$CONT_PWD_AUTO_SOURCE_SETUP" == true ]] ; then
    if [ -f "$CONT_PWD/setup_devcont.sh" ] ; then
        . "$CONT_PWD/setup_devcont.sh"
    fi
fi


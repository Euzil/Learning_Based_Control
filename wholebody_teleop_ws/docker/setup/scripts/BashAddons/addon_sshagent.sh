#!/bin/bash

sshagent()
{
    if [ "$#" -eq 0 ];
        then
        #echoc "exec ssh-agent bash"
        echoc BYELLOW "Starting ssh-agent"
        eval `ssh-agent`
        #echoc "ssh-add"
        ssh-add
    fi

    if [ "$1" == "-d" ];
    then
        echoc BYELLOW "Removing ssh-agent"
        ssh-add -D
    fi
}

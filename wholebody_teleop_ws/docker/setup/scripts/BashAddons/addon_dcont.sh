#!/bin/bash

dcont_bash()
{
    local CONT_NAME=$1
    #docker exec -it $CONT_NAME runuser -u devel -- bash -c "cd \$CONT_PWD && bash"
    docker exec -it $CONT_NAME runuser -u `id -u -n` -- bash -c "cd \$CONT_PWD && export CONT_PWD_AUTO_SOURCE_SETUP=true && bash"
}

dcont_bash_complete() 
{
    local contNames=($(docker ps --format "{{.Names}}"))

    COMPREPLY=()
    
    local word="${COMP_WORDS[COMP_CWORD]}"
    
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=($(compgen -W "${contNames[*]}" -- "$word"));
    fi
}

complete -F dcont_bash_complete dcont_bash 

#!/bin/bash

devcont_bash()
{
    local CONT_NAME=$1
    #docker exec -it $CONT_NAME runuser -u devel -- bash -c "cd \$CONT_PWD && bash"
    docker exec -it $CONT_NAME runuser -u devel -- bash -c "cd \$CONT_PWD && export CONT_PWD_AUTO_SOURCE_SETUP=true && bash"
}

devcont_bash_complete() 
{
    local contNames=($(docker ps --format "{{.Names}}"))

    COMPREPLY=()
    
    local word="${COMP_WORDS[COMP_CWORD]}"
    
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=($(compgen -W "${contNames[*]}" -- "$word"));
    fi
}

complete -F devcont_bash_complete devcont_bash 

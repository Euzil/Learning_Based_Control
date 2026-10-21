#!/bin/bash


path_remove()  
{ 
    export PATH=`echo -n $PATH | awk -v RS=: -v ORS=: '$0 != "'$1'"' | sed 's/:$//'` 
}

path_append()  
{
    path_remove $1
    export PATH="$PATH:$1"
}

path_prepend()
{ 
    path_remove $1
    export PATH="$1:$PATH"
}

path_print()
{
    echo $PATH
}

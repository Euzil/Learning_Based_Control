#!/bin/bash

upenv()
{
    source $HOME/.bashrc
}

sups()
{
    if [ -v CONT_PWD ] ; then
        if [ -f "setup_devcont.sh" ] ; then
            . "setup_devcont.sh"
        fi
        return
    fi
    
    . setup.sh
}


# rcp [params] <src-dir> <dst-dir>/
#   Examples:
#   
#   rcp --dry-run current/. save_yyyymmdd
#   rcp current/. save_yyyymmdd
#
#   rcp --dry-run current/. save_$(date +%Y%m%d)
#   rcp current/. save_$(date +%Y%m%d)
#
rcp()
{ 
    sudo rsync -avh --progress --stats --numeric-ids $@
}


use_nvidia_run()
{
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia $@
}

use_nvidia_set()
{
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
}

use_nvidia_unset()
{
    unset __NV_PRIME_RENDER_OFFLOAD
    unset __GLX_VENDOR_LIBRARY_NAME
}

use_nvidia_info()
{
    export | grep -i __NV_PRIME_RENDER_OFFLOAD
    export | grep -i __GLX_VENDOR_LIBRARY_NAME
}

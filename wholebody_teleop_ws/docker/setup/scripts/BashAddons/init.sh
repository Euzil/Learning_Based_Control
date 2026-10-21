#!/bin/bash

bash_addons_init()
{
    local sourceDir=$BASH_SOURCE
    local runDir=$0
    local scriptDir=$(cd $(dirname $sourceDir); pwd)
    local scriptName=$(basename -- "$BASH_SOURCE")

    # echo "Bash addons dir: $scriptDir"

    local addons=($(find $scriptDir -type f -name "addon_*.sh"))
    local a
    for a in ${addons[@]}
    do
        # echo $a
        source $a
    done
}

bash_addons_init




#!/bin/bash

GT_DRY_RUN=false
GT_SETUP_OK=false


# names of remotes
gt_setup_remote_names()
{
    local pyScript=$(cat \
 <<-END
import sys
import json
d = json.load(sys.stdin)['remotes']
for e in d:
    print(e['name'])
END
)
    local ret
    ret=$(echo "$GT_CONFIG_JSON" | python3 -c "$pyScript")

    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not get remote names." 1>&2
        return 1
    fi

    GT_REMOTE_NAMES=($ret)
}


# base urls of remotes
gt_setup_remote_urls()
{
    pyScript=$(cat \
 <<-END
import sys
import json
d = json.load(sys.stdin)['remotes']
for e in d:
    print(e['base_url'])
END
)
    local ret
    ret=$(echo "$GT_CONFIG_JSON" | python3 -c "$pyScript")
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not get remote base URLs." 1>&2
        return 1
    fi

    GT_REMOTE_URLS=($ret) 
}

# get mode, default to "default"
gt_setup_mode()
{
    pyScript=$(cat \
 <<-END
import sys
import json
d = json.load(sys.stdin)
try:
    d = d['mode']
except KeyError:
    print("default")
    sys.exit(0)
print(d)
END
)
    local ret
    ret=$(echo "$GT_CONFIG_JSON" | python3 -c "$pyScript")
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not get mode." 1>&2
        return 1
    fi

    GT_MODE="$ret"
}


# get remote paths, defaults to ""
gt_setup_remote_paths()
{
    pyScript=$(cat \
 <<-END
import sys
import json
d = json.load(sys.stdin)
try:
    d = d['remote_paths']
    for e in d:
        print(e)
except KeyError:
    print("")
    sys.exit(0)
END
)
    local ret
    ret=$(echo "$GT_CONFIG_JSON" | python3 -c "$pyScript")
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not get remote_paths." 1>&2
        return 1
    fi

    GT_REMOTE_PATHS=($ret)
    # local paths=()

    # for p in ${GT_REMOTE_PATHS[@]} ; do
    #     #echo $p
    #     paths+=("$p")
    # done

    # GT_MODULE_PATHS=(${p[@]})
}


# get relative module paths
# default mode: transparent (project = remote structure)
# project mode: src/mpath = remote/mpath
gt_setup_module_paths()
{
    pyScript=$(cat \
 <<-END
import sys
import json
d = json.load(sys.stdin)['module_paths']
for e in d:
    print(e)
END
)
    local ret
    ret=$(echo "$GT_CONFIG_JSON" | python3 -c "$pyScript")
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not get module paths." 1>&2
        return 1
    fi

    GT_MODULE_PATHS=($ret)

    local modules=()

    for m in ${GT_MODULE_PATHS[@]} ; do
        #echo $m
        if [[ "${m:0:1}" == ":" ]] ; then
            local temp=($(cat "${m:1}"))
            for mm in ${temp[@]} ; do
                modules+=("$mm")
            done
        else
            modules+=("$m")
        fi
    done

    GT_MODULE_PATHS=(${modules[@]})
}


gt_setup_config()
{
    GT_SETUP_OK=false
    GT_WORK_DIR="$(pwd)"
    GT_PROJECT_NAME="$(basename "$GT_WORK_DIR")"
    GT_CONFIG_FILE="$GT_WORK_DIR/configs.json"

    if [ ! -f "$GT_CONFIG_FILE" ] ; then
        echo "ERROR: Could not load config file" 1>&2 
        echo " $GT_CONFIG_FILE" 1>&2
        echo "in" 1>&2
        echo " $GT_WORK_DIR" 1>&2
        return 1
    fi

    GT_CONFIG_JSON=$(cat "$GT_CONFIG_FILE")

    gt_setup_mode &&
        gt_setup_remote_names &&
        gt_setup_remote_urls &&
        gt_setup_module_paths &&
        gt_setup_remote_paths
    
    if [ ! $? -eq 0 ] ; then
        return 1
    fi
    
    GT_SETUP_OK=true
}


gt_info()
{
    echo "GT_WORK_DIR"
    echo " $GT_WORK_DIR"

    echo "GT_PROJECT_NAME"
    echo " $GT_PROJECT_NAME"

    echo "GT_MODE"
    echo " $GT_MODE"

    echo "GT_REMOTE_NAMES"
    for i in ${GT_REMOTE_NAMES[@]}; do
        echo " $i"
    done

    echo "GT_REMOTE_URLS"
    for i in ${GT_REMOTE_URLS[@]}; do
        echo " $i"
    done

    if [ -n "$GT_REMOTE_PATHS" ] ; then
        echo "GT_REMOTE_PATHS"
        for i in ${GT_REMOTE_PATHS[@]}; do
            echo " $i"
        done
    fi

    echo "GT_MODULE_PATHS"
    for i in ${GT_MODULE_PATHS[@]}; do
        echo " $i"
    done
}


gt_setup()
{
    complete -F gt_complete gt
    
    if ! gt_setup_config ; then
        return 1
    fi   

    gt_info
}   


# get base url for given remote name
gt_base_url()
{
    local remote=$1
    local len=${#GT_REMOTE_NAMES[@]}

    for (( i=0; i<len; i++ ))
    do
        local name=${GT_REMOTE_NAMES[$i]}

        if [[ "$name" == "$remote" ]] ; then
            echo ${GT_REMOTE_URLS[$i]}
            return 0
        fi
    done

    return 1
}


# take module path and create module url depending on
#   remote
# Usage:
#   gt_module_url <module-path> <remote>
gt_module_url()
{
    local modulePath=$1
    local remote=$2
    local ret

    ret=$(gt_base_url $remote)
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not find base URL for remote '$remote'."
        return 1
    fi

    local baseUrl="$ret"
    local url

    case $GT_MODE in
        default)
        url="$baseUrl/$modulePath.git"
        ;;

        project)    
        url="$baseUrl/Packages/$modulePath.git"
        ;;
        
        project2)    
        url="$baseUrl/packages/$modulePath.git"
        ;;


        ros-workspace)  
        
        url="$baseUrl/Packages/$modulePath.git"

        if [ -n "$GT_REMOTE_PATHS" ] ; then
            local baseDir
            baseDir=$(dirname "${modulePath}")
            
            # if there is no base dir then the module itself is the base dir
            if [[ "$baseDir" == "." ]] ; then
                baseDir=$(basename "${modulePath}" ".git")
            fi 

            for p in ${GT_REMOTE_PATHS[@]} ; do
                local s
                s=(${p//':'/ })
                local remotePrefix=${s[0]}
                local moduleBaseDir=${s[1]}
                if [[ "$baseDir" == "$moduleBaseDir" ]] ; then
                    url="$baseUrl/$remotePrefix/Packages/$modulePath.git"
                    echo "$url"
                    return 0
                fi
            done

            echo "ERROR: Missing module base dir '$baseDir' in 'remote_paths'" 1>&2
            return 1
        fi
        ;;

        ros-workspace2)  
        
        url="$baseUrl/packages/$modulePath.git"

        if [ -n "$GT_REMOTE_PATHS" ] ; then
            local baseDir
            baseDir=$(dirname "${modulePath}")
            
            # if there is no base dir then the module itself is the base dir
            if [[ "$baseDir" == "." ]] ; then
                baseDir=$(basename "${modulePath}" ".git")
            fi 

            for p in ${GT_REMOTE_PATHS[@]} ; do
                local s
                s=(${p//':'/ })
                local remotePrefix=${s[0]}
                local moduleBaseDir=${s[1]}
                if [[ "$baseDir" == "$moduleBaseDir" ]] ; then
                    url="$baseUrl/$remotePrefix/packages/$modulePath.git"
                    echo "$url"
                    return 0
                fi
            done

            echo "ERROR: Missing module base dir '$baseDir' in 'remote_paths'" 1>&2
            return 1
        fi
        ;;

        *) # unknown mode
        echo "ERROR: Unknown mode '$GT_MODE'" 1>&2
        return 1
        ;;
    esac

    echo "$url"
}


# create the url for the main project/repository
# Usage:
#   gt_project_url <remote>
gt_project_url()
{
    local remote=$1
    local ret

    ret=$(gt_base_url $remote)
    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not find base URL for remote '$remote'."
        return 1
    fi

    local baseUrl="$ret"
    local url

    case $GT_MODE in
        default)
        url="$baseUrl/$GT_PROJECT_NAME.git"
        ;;

        project)    
        url="$baseUrl/Projects/$GT_PROJECT_NAME.git"
        ;;
        
        project2)    
        url="$baseUrl/projects/$GT_PROJECT_NAME.git"
        ;;


        ros-workspace)    
        url="$baseUrl/Workspaces/$GT_PROJECT_NAME.git"
        ;;
        
        ros-workspace2)    
        url="$baseUrl/workspaces/$GT_PROJECT_NAME.git"
        ;;

        *) # unknown mode
        echo "ERROR: Unknown mode '$GT_MODE'" 1>&2
        return 1
        ;;
    esac

    echo "$url"
}


# take module path and create module destination path
# Usage:
#   gt_module_dest_path <module-path>
gt_module_dest_path()
{
    local modulePath=$1
    local ret

    local dst

    case $GT_MODE in
        default)
        #url="$baseUrl/$modulePath.git"
        dst="$modulePath"
        ;;

        project)    
        #url="$baseUrl/Packages/$modulePath.git"
        dst="src/$modulePath"
        ;;
        
        project2)    
        #url="$baseUrl/Packages/$modulePath.git"
        dst="src/$modulePath"
        ;;

        ros-workspace)
        #url="$baseUrl/Packages/$modulePath.git"
        dst="src/$modulePath"
        ;;
        
        ros-workspace2)
        #url="$baseUrl/Packages/$modulePath.git"
        dst="src/$modulePath"
        ;;

        *) # unknown option
        echo "ERROR: Unknown mode '$GT_MODE'" 1>&2
        return 1
        ;;
    esac

    echo "$dst"
}


gt_add_modules()
{
    local remote=$1
    local ret

    ret=$(gt_base_url $remote)

    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not find base URL for remote '$remote'."
        return 1
    fi

    local baseUrl=$ret

    for modulePath in ${GT_MODULE_PATHS[@]}
    do
        local url=
        local dst=

        url="$(gt_module_url "$modulePath" "$remote")"
        dst="$(gt_module_dest_path "$modulePath")"

        local cmd="echo \"1\" | gitsadd $url $dst"
        
        if [ -z $GT_DRY_RUN ] ; then 
            echo $cmd
            continue
        fi

        if [[ "$GT_DRY_RUN" == true ]] ; then
            echo $cmd
            continue    
        fi   

        eval "$cmd"            

    done
}


gt_clone_modules()
{
    local remote=$1
    local ret

    ret=$(gt_base_url $remote)

    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not find base URL for remote '$remote'."
        return 1
    fi

    local baseUrl=$ret

    for modulePath in ${GT_MODULE_PATHS[@]}
    do
        local url=
        local dst=

        url="$(gt_module_url "$modulePath" "$remote")"
        dst="$(gt_module_dest_path "$modulePath")"

        local cmd="gitclone $url $dst"
        
        if [ -z $GT_DRY_RUN ] ; then 
            echo $cmd
            continue
        fi

        if [[ "$GT_DRY_RUN" == true ]] ; then
            echo $cmd
            continue    
        fi   

        eval "$cmd" 

    done
}


gt_add_remotes()
{
    local remote=$1
    local ret

    ret=$(gt_base_url $remote)

    if [ ! $? -eq 0 ] ; then
        echo "ERROR: Could not find base URL for remote '$remote'."
        return 1
    fi

    local baseUrl=$ret

    for modulePath in ${GT_MODULE_PATHS[@]}
    do
        local url=
        local dst=

        url="$(gt_module_url "$modulePath" "$remote")"
        dst="$(gt_module_dest_path "$modulePath")"

        local moduleDirPath="$GT_WORK_DIR/$dst"
                
        if [ ! -d "$moduleDirPath" ] ; then
            continue
        fi
        
        local cmd="git remote add $remote $url"

        echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"

        if [ -z $GT_DRY_RUN ] ; then
            # echo $moduleDirPath 
            # echo $cmd
            # echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"
            continue
        fi

        if [[ "$GT_DRY_RUN" == true ]] ; then
            # echo $moduleDirPath 
            # echo $cmd
            # echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"
            continue    
        fi   
        
        # sub shell
        ret=$(
            cd "$moduleDirPath"
            # echo $cmd

            # echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"

            eval "$cmd" 2>&1
        )
        
        echo "$ret"

    done

    url="$(gt_project_url "$remote" )"
    cmd="git remote add $remote $url"

    echoc LYELLOW "$cmd in " LBLUE "$GT_WORK_DIR"
    eval "$cmd"
}


gt_modules_do()
{
    #local args="$@"

    local baseUrl=$ret

    for modulePath in ${GT_MODULE_PATHS[@]}
    do
        local dst=
        dst="$(gt_module_dest_path "$modulePath")"
        local moduleDirPath="$GT_WORK_DIR/$dst"
        
        local cmd="$@"
        
        echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"

        if [ -z $GT_DRY_RUN ] ; then
            # echo $moduleDirPath
            # echo $cmd
            # echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"
            continue
        fi

        if [[ "$GT_DRY_RUN" == true ]] ; then
            #echo $moduleDirPath
            #echo $cmd
            # echoc LYELLOW "$cmd in " LBLUE "$moduleDirPath"  

            continue    
        fi   
    

        local ret
        
        # sub shell
        ret=$(
            cd "$moduleDirPath"
            #pwd
            #echo $cmd
            # echoc LYELLOW "$cmd in " LBLUE "$(pwd)"

            "$@" 2>&1
        )
        
        echo "$ret"

    done
}


gt_reload()
{
    source gtsetup
}


gt_complete() 
{
    COMPREPLY=()
    
    local cmds
    
    cmds=()
    cmds+=("info")
    cmds+=("setup")
    cmds+=("reload")
    cmds+=("add")
    cmds+=("clone")
    cmds+=("modules")

    local addOpts=()
    addOpts+=("modules")
    addOpts+=("remotes")

    local cloneOpts=()
    cloneOpts+=("modules")

    local modulesOpts=()
    modulesOpts+=("do")


    local remoteOpts=(${GT_REMOTE_NAMES[@]})   
    local word="${COMP_WORDS[COMP_CWORD]}";
    
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=($(compgen -W "${cmds[*]}" -- "$word"));
    fi

    if [ "$COMP_CWORD" -eq 2 ]; then
        local words=("${COMP_WORDS[@]}");
        
        if [[ "${words[1]}" == "add" ]] ; then
            COMPREPLY=($(compgen -W "${addOpts[*]}" -- "$word"));
            return
        fi

        if [[ "${words[1]}" == "clone" ]] ; then
            COMPREPLY=($(compgen -W "${cloneOpts[*]}" -- "$word"));
            return
        fi

        if [[ "${words[1]}" == "modules" ]] ; then
            COMPREPLY=($(compgen -W "${modulesOpts[*]}" -- "$word"));
            return
        fi
    fi
    
    if [ "$COMP_CWORD" -eq 3 ]; then
        local words=("${COMP_WORDS[@]}");
        
        if [[ "${words[1]}" == "add" ]] && [[ "${words[2]}" == "modules" ]]  ; then           
            COMPREPLY=($(compgen -W "${remoteOpts[*]}" -- "$word"));
            return
        fi
        
        if [[ "${words[1]}" == "add" ]] && [[ "${words[2]}" == "remotes" ]]  ; then           
            COMPREPLY=($(compgen -W "${remoteOpts[*]}" -- "$word"));
            return
        fi

        if [[ "${words[1]}" == "clone" ]] && [[ "${words[2]}" == "modules" ]]  ; then           
            COMPREPLY=($(compgen -W "${remoteOpts[*]}" -- "$word"));
            return
        fi
    fi
    
}


gt()
{
    local opt1=$1
    local opt2=$2
    
    if [[ "$opt1" == "setup" ]] ; then
        gt_setup
        return
    fi
    
    if [[ "$opt1" == "reload" ]] ; then
        gt_reload
        return
    fi
    
    if [[ "$GT_SETUP_OK" != true ]] ; then
        echo "ERROR: Setup not OK. Run 'gt setup'." 1>&2
        return 1
    fi
        

    if [[ "$opt1" == "add" ]] ; then

        local remote="origin"           
        if [ $# -eq 3 ] ; then
            remote="$3"
        fi
    
        if [[ "$opt2" == "modules" ]] ; then
            gt_add_modules $remote
            return           
        fi
        
        if [[ "$opt2" == "remotes" ]] ; then
            gt_add_remotes $remote
            return           
        fi
    fi

    if [[ "$opt1" == "clone" ]] ; then

        local remote="origin"           
        if [ $# -eq 3 ] ; then
            remote="$3"
        fi
    
        if [[ "$opt2" == "modules" ]] ; then
            gt_clone_modules $remote
            return           
        fi
    fi

    if [[ "$opt1" == "modules" ]] ; then
        if [[ "$opt2" == "do" ]] ; then
            #local args=($@)
            #args=${args[@]:2}
            gt_modules_do "${@:3}"
            return           
        fi
    fi

    
    if [[ "$opt1" == "info" ]] ; then
        gt_info
        return
    fi
   
    echo "gt $@"
    echo "ERROR: unknown command." 1>&2
}

###############################################################################

complete -F gt_complete gt

# gt_setup_config

# echo "GT_MODE"
# echo " $GT_MODE"

# echo "GT_REMOTE_NAMES"
# for i in ${GT_REMOTE_NAMES[@]}; do
#     echo " $i"
# done

# echo "GT_REMOTE_URLS"
# for i in ${GT_REMOTE_URLS[@]}; do
#     echo " $i"
# done

# echo "GT_MODULE_PATHS"
# for i in ${GT_MODULE_PATHS[@]}; do
#     echo " $i"
# done

# gt_setup

# return 0

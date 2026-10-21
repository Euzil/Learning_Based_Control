#!/bin/bash

gitst()
{
    echoc YELLOW "GIT STATUS " LBLUE "${PWD}" 
    git status
}

gitrst()
{
    local status=$(gitst)

    if [[ $status == *"git add"* ]] ; then
        git submodule foreach --recursive --quiet bash -ci \
            "[[ \$(gitst) == *\"git add\"* ]] && gitst || :"

        gitst
        return
    fi

    echoc LCYAN "NO CHANGES!"
    gitst  
}

gitlog()
{
    echoc YELLOW "GIT LOG " LBLUE "${PWD}"
    git log
}

giturl()
{
    echoc YELLOW "GIT REMOTE REPOSITORIES " LBLUE "${PWD}"
    git remote -v
}

gitup()
{
    if [ "$#" -eq 0 ]; then
        1>&2 echo "Error: At least one input argument required."
        return 1
    fi
    
    if [ "$#" -eq 1 ]; then
        echoc YELLOW "GIT ADD ALL " LBLUE "${PWD}" \
        && git add --all \
        && echoc YELLOW "GIT COMMIT " LBLUE "${PWD}" \
        && git commit -m "$1" \
        && echoc YELLOW "GIT PUSH " LBLUE "${PWD}" \
        && git push \
        && echoc YELLOW "GIT STATUS " LBLUE "${PWD}" \
        && git status 

        # git add --all && git commit -m "$1" && git push
        return
    fi
        
    if [ "$#" -eq 3 ]; then
        echoc YELLOW "GIT ADD ALL " LBLUE "${PWD}" \
        && git add --all \
        && echoc YELLOW "GIT COMMIT " LBLUE "${PWD}" \
        && git commit -m "$3" \
        && echoc YELLOW "GIT PUSH " LBLUE "${PWD}" \
        && git push $1 $2 \
        && echoc YELLOW "GIT STATUS " LBLUE "${PWD}" \
        && git status
        
        # git add --all && git commit -m "$3" && git push $1 $2
        return 
    fi
    
    1>&2 echo "Error: Invalid number of input arguments."
    return
}

gitrup()
{
    local status=$(gitst)

    local args
    args=''
    for i in "$@"; do 
        args="$args ${i@Q}"
    done

    if [[ $status == *"git add"* ]] ; then
        git submodule foreach --recursive --quiet bash -ci \
            "[[ \$(gitst) == *\"git add\"* ]] && gitup $args || :"

        gitup "$@"
        return
    fi

    echoc LCYAN "NO CHANGES!"
    gitst
}


gitclone()
{
    git clone $@
}

gitpush()
{
    echoc YELLOW "GIT PUSH " LBLUE "${PWD}"
    echoc GREEN "git push $*"
    git push $@
}

gitrpush()
{
    echoc YELLOW "GIT PUSH RECURSIVE " LBLUE "${PWD}"
    echoc GREEN "gitrdo push $*"
    gitrdo push $@

    echoc GREEN "git push $*"
    git push $@
}

gitpull()
{
    echoc YELLOW "GIT PULL " LBLUE "${PWD}"
    echoc GREEN "git pull $*"
    git pull $@
}

gitrpull()
{
    echoc YELLOW "GIT PULL RECURSIVE" LBLUE "${PWD}"

    echoc GREEN "gitrdo pull $*"
    gitrdo pull $@

    echoc GREEN "git pull $*"
    git pull $@
}

# gitrdo()
# {
#     # loop even if one command fails
#     local cmd="$@ || :"
# 	 git submodule foreach --recursive "git $cmd"
# }

# gitrdo2()
# {
#     # loop even if one command fails
#     local cmd="$@ || :"
# 	git submodule foreach --recursive "$cmd"
# }


gitrdo()
{
    local args=''
    for i in "$@"; do 
        args="$args ${i@Q}"
    done

	git submodule foreach --recursive bash -ci "git $args || :"
}

gitrdo2()
{
    local args=''
    for i in "$@"; do 
        args="$args ${i@Q}"
    done

	git submodule foreach --recursive bash -ci "$args || :"
}

gitsiniup()
{
    git submodule update --init
}

gitsiniupcom()
{
    # git submodule init
    # git submodule update
    git submodule update --init &&
    gitrdo checkout master
}

gitsiniupcom2()
{
    # git submodule init
    # git submodule update
    git submodule update --init &&
    gitrdo checkout main
}

gitsadd_help()
{
  	echoc BWHITE "NAME"
	echoc NOCOLOR "    gitsadd"
	echoc BWHITE "SYNOPSIS"
	echoc NOCOLOR "    gitsadd " YELLOW "[options] " LBLUE "<project_url> " LGREEN "[destination_path]"
	echoc BWHITE "DESCRIPTION"
	echoc NOCOLOR "	The family of functions git(" BWHITE "s" NOCOLOR ")* are standard git commands for " BWHITE "submodules." 
	echoc BWHITE "        gitsadd " NOCOLOR "is an alias for: " BWHITE "git submodule add " LBLUE "project_url " LGREEN "destination_path " NOCOLOR "followed by " BWHITE "gitst " NOCOLOR "command."
	echoc NOCOLOR "	If " BWHITE "destination_path " NOCOLOR "is not provided, then gitsadd will use the name of the git repo (without extension) as the destination dir."
	echoc NOCOLOR "        If " BYELLOW "-d " NOCOLOR "is defined in " YELLOW "options " NOCOLOR "then " LGREEN "destination_path " NOCOLOR "will be appended to the git repo name (whitout extension) to define the destination dir. e.g." 
	echoc NOCOLOR "        " BWHITE "gitsadd " BBLUE "ssh://repo_path/gitrepo_name.git " NOCOLOR "will add the submodule " BBLUE "ssh://repo_path/gitrepo_name.git " NOCOLOR "into the dir: " BBLUE "gitrepo_name"
	echoc NOCOLOR "        " BWHITE "gitsadd " BBLUE "ssh://repo_path/gitrepo_name.git " BGREEN "path_name " NOCOLOR "will add the submodule " BBLUE "ssh://repo_path/gitrepo_name.git " NOCOLOR "into the dir: " BBLUE "path_name"
	echoc NOCOLOR "        " BWHITE "gitsadd " BYELLOW "-d " BBLUE "ssh://repo_path/gitrepo_name.git " BGREEN "path " NOCOLOR "will add the submodule " BBLUE "ssh://repo_path/gitrepo_name.git " NOCOLOR "into the dir: " BGREEN "path" BBLUE "/gitrepo_name"
	echoc BWHITE "AUTHORS"
	echoc NOCOLOR "    Emmanuel Dean <dean@tum.de>"
	echoc NOCOLOR "    Florian Bergner <florian.bergner@tum.de>"
}


gitsadd()
{
    if [ "$1" == "-h" ] || [ "$1" == "--help" ] ; then
        gitsadd_help
        return 1
    fi

    if [ $# -eq 0 ] || [ $# -gt 3 ] ; then
        echoc BLRED "ERROR: Wrong number of arguments. See " BWHITE "gitsadd --help"
        gitsadd_help
        return 1
    fi

    if [ $# -eq 3 ] ; then
        if [ "$1" == "-d" ] ; then
            repo="$2"
            repo_ssh=${repo/"sftp:"/"ssh:"}
            location=${repo_ssh%.*}
            destdir="${3}/"$(basename $location)
        else
            echoc BLRED "ERROR: Wrong use of arguments. See " BWHITE "gitsadd --help"
            gitsadd_help
            return 1
        fi
    else
        if [ "$1" == "-d" ] ; then
            echoc BLRED "ERROR: Wrong use of arguments. See " BWHITE "gitsadd --help"
            gitsadd_help
            return 1
        fi

        #Add submodule
        repo="$1"

        # replace the first occurence of the patterns "sftp" with "ssh"
        repo_ssh=${repo/"sftp:"/"ssh:"}
        #echo "$repo" "$repo_ssh"
        
        if [ $# -eq 1 ] ; then
            location=${repo_ssh%.*}
            destdir=$(basename $location)
        else
            destdir="${2}"
        fi
    fi

    echoc BYELLOW "Adding submodule: " BBLUE "${repo_ssh} " BYELLOW "into the dir " BGREEN "${destdir}" 
    echo "Do you still want to add this submodule?"
    select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) echoc BLRED "The new submodule was not added!!"; return;;
    esac
    done

    git submodule add "${repo_ssh}" "${destdir}"
    #git submodule add "$1" "$2"
    gitst
}

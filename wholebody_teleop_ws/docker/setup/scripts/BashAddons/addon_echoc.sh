#!/bin/bash

# NEED colors

echoc_help()
{
	echo -e "${BLGRAY}NAME${NOCOLOR}"
	echo "    echoc"
	echo -e "${BCOLOR}SYNOPSIS${NOCOLOR}"
	echo "    echoc [color] <msg>"
	echo -e "${BCOLOR}DESCRIPTION${NOCOLOR}"
	echo -e "    echoc prints string in msg using color." 
	echo -e "    The colors are based on the color codes found in '.bash_colors'." 
	echo -e "    You can concatenate different msg using different colors, for example: "
  	echo -e "    \$${BWHITE}echoc $RED COLOR1 $YELLOW msg1 $RED COLOR2 $YELLOW msg2 $NOCOLOR"
    echo -e "    Run ${BWHITE}showcolors$NOCOLOR to see a list of available colors."
	echo -e "${BCOLOR}AUTHORS${NOCOLOR}"
	echo -e "    Emmanuel Dean <dean@tum.de>"
	echo -e "    Florian Bergner <florian.bergner@tum.de>"
}


echoc()
{
    if [ "$1" == "-h" ] || [ "$1" == "--help" ] ; then
	    echoc_help
	    return
    fi

    if [ $# -eq 0 ] ; then
	     echo -e "${BLRED}ERROR: No arguments supplied. See: ${BWHITE}echoc --help ${NOCOLOR}"
	     echoc_help
	     return 1
    fi

    if [ $# -eq 1 ]; then
	    local msg="$1"
	    echo -e "${NOCOLOR}${msg}${NOCOLOR}"
    else
	    if [ $(( $# % 2)) -eq 0 ] ; then
            local s="echo -e \""
            
            local i=
            for ((i=1;i<$#;i+=2)); do
                 local gc='${'"${i}"'}'
                 local color='${'$(eval "echo  \"$gc\"")'}'
                 
                 local idx=$((i+1))
                 local gm='${'"${idx}"'}'
                 local msg=$(eval "echo  \"$gm\"")""
                 gm=""
                
                 s+="$color"
                 s+="$msg"
            done
            
            color_end='$'"NOCOLOR"
            s+=$color_end
            s+="\""

            eval "$s"
        else
            echo -e "${BRED}ERROR: Wrong number of arguments." 
            echo -e "  Arguments must be given in ${BYELLOW}pairs. ${BRED}See: ${BWHITE}echoc --help ${NOCOLOR}"
            echoc_help
            return 1
        fi 
    fi
}


showcolors()
{
    local color=("GRAY" "RED" "GREEN" "YELLOW" "BLUE" "PURPLE" "CYAN" "WHITE")
    local lcolor=("LGRAY" "LRED" "LGREEN" "LYELLOW" "LBLUE" "LPURPLE" "LCYAN" "LWHITE")
    local prefix=("" "B" "D" "U" "BA")

    for i in "${prefix[@]}"
    do
    
        echoc WHITE "------------" BWHITE "*${i}*"  WHITE "------------"
        for ((j=0;j<${#color[@]};j++)) 
        do
            echoc "${i}${color[j]}" "$i${color[j]}" WHITE "\t  " "${i}${lcolor[j]}" "${i}${lcolor[j]}"  
        done
    done
}






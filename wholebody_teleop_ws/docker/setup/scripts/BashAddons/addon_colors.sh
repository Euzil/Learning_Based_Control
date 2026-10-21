#!/bin/bash


# COLORS CODES

#NOCOLOR='\033[0m'
NOCOLOR='\e[0m'
BCOLOR='\e[1m'

#Nomenclature:
#	B: Bold
#	D: dark
#	U: underlined
#	BA:colored backgorund
#	L: light

GRAY='\e[0;49;30m' 
BGRAY='\e[1;49;30m' 
DGRAY='\e[2;49;30m' 
UGRAY='\e[4;49;30m' 
BAGRAY='\e[7;49;30m' 

RED='\e[0;49;31m' 
BRED='\e[1;49;31m' 
DRED='\e[2;49;31m' 
URED='\e[4;49;31m' 
BARED='\e[7;49;31m' 

GREEN='\e[0;49;32m' 
BGREEN='\e[1;49;32m' 
DGREEN='\e[2;49;32m' 
UGREEN='\e[4;49;32m' 
BAGREEN='\e[7;49;32m' 

YELLOW='\e[0;49;33m' 
BYELLOW='\e[1;49;33m' 
DYELLOW='\e[2;49;33m' 
UYELLOW='\e[4;49;33m' 
BAYELLOW='\e[7;49;33m' 

BLUE='\e[0;49;34m' 
BBLUE='\e[1;49;34m' 
DBLUE='\e[2;49;34m' 
UBLUE='\e[4;49;34m' 
BABLUE='\e[7;49;34m' 

PURPLE='\e[0;49;35m' 
BPURPLE='\e[1;49;35m' 
DPURPLE='\e[2;49;35m' 
UPURPLE='\e[4;49;35m' 
BAPURPLE='\e[7;49;35m' 

CYAN='\e[0;49;36m' 
BCYAN='\e[1;49;36m' 
DCYAN='\e[2;49;36m' 
UCYAN='\e[4;49;36m' 
BACYAN='\e[7;49;36m' 

WHITE='\e[0;49;37m' 
BWHITE='\e[1;49;37m' 
DWHITE='\e[2;49;37m' 
UWHITE='\e[4;49;37m' 
BAWHITE='\e[7;49;37m' 

LGRAY='\e[0;49;39m' 
BLGRAY='\e[1;49;39m' 
DLGRAY='\e[2;49;39m' 
ULGRAY='\e[4;49;39m' 
BALGRAY='\e[7;49;39m' 

LRED='\e[0;49;91m' 
BLRED='\e[1;49;91m' 
DLRED='\e[2;49;91m' 
ULRED='\e[4;49;91m' 
BALRED='\e[7;49;91m' 

LGREEN='\e[0;49;92m' 
BLGREEN='\e[1;49;92m' 
DLGREEN='\e[2;49;92m' 
ULGREEN='\e[4;49;92m' 
BALGREEN='\e[7;49;92m' 

LYELLOW='\e[0;49;93m' 
BLYELLOW='\e[1;49;93m' 
DLYELLOW='\e[2;49;93m' 
ULYELLOW='\e[4;49;93m' 
BALYELLOW='\e[7;49;93m' 

LBLUE='\e[0;49;94m' 
BLBLUE='\e[1;49;94m' 
DLBLUE='\e[2;49;94m' 
ULBLUE='\e[4;49;94m' 
BALBLUE='\e[7;49;94m' 

LPURPLE='\e[0;49;95m' 
BLPURPLE='\e[1;49;95m' 
DLPURPLE='\e[2;49;95m' 
ULPURPLE='\e[4;49;95m' 
BALPURPLE='\e[7;49;95m' 

LCYAN='\e[0;49;96m' 
BLCYAN='\e[1;49;96m' 
DLCYAN='\e[2;49;96m' 
ULCYAN='\e[4;49;96m' 
BALCYAN='\e[7;49;96m' 

LWHITE='\e[0;49;97m' 
BLWHITE='\e[1;49;97m' 
DLWHITE='\e[2;49;97m' 
ULWHITE='\e[4;49;97m' 
BALWHITE='\e[7;49;97m' 




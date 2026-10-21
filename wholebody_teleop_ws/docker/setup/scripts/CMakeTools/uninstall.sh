#!/bin/bash

currentDir="$(pwd)"
echo ${currentDir}

if [ ! -d build ]; then
	echo "ERROR: no build dir"	
	exit 1
fi

cd build
sudo make uninstall
cd $currentDir
rm -rf build




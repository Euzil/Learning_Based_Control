#!/bin/bash

currentDir="$(pwd)"
echo ${currentDir}

buildDir=build-local

if [ -d $buildDir ]; then
    cd $buildDir
    make uninstall
    cd $currentDir
    rm -rf $buildDir
fi

mkdir $buildDir
cd $buildDir
cmake .. -DCMAKE_INSTALL_PREFIX=$currentDir/install
make install
cd $currentDir



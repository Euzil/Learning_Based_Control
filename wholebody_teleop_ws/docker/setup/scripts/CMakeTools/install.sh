#!/bin/bash

currentDir="$(pwd)"
echo ${currentDir}

if [ -d build ]; then
	cd build
	sudo make uninstall
	cd $currentDir
	rm -rf build
fi

instPrefix=/usr

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${instPrefix} ..
sudo make install
cd $currentDir



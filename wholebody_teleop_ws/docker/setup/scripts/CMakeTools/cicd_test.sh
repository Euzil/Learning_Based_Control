#!/bin/bash

GL_CICD_INIT_URI="ssh://git@gitlab.flocl.de:2220/share/GitLabCICDInit.git"
GL_CICD_TOOLS_URI="ssh://git@gitlab.flocl.de:2220/cicd/GitLabCICDTools.git"
GL_CICD_SSH_PRIVATE_KEY=$(cat $HOME/.ssh/flocl_cicd_r)


# gitlab cicd sets this option by default
set -e

TMP_DIR="$(pwd)/temp"

rm -rf $TMP_DIR
# rm -rf GitLabCICDInit
# rm -rf GitLabCICDTools

mkdir -p $TMP_DIR

git clone $GL_CICD_INIT_URI $TMP_DIR/GitLabCICDInit
source $TMP_DIR/GitLabCICDInit/build_setup.sh

fbc_build_setup \
	--targetdir $TMP_DIR

fbc_cmake_build --prefix usr \
	--workdir $TMP_DIR

fbc_deb_build --config deb/config.sh \
	--sourcedir $TMP_DIR/install \
	--targetdir $TMP_DIR

# fbc_pkg_cks $TMP_DIR/*.deb
# fbc_pkg_cks $TMP_DIR/install

# mkdir -p $TMP_DIR/deb-temp
# fbc_deb_cks $TMP_DIR/*.deb $TMP_DIR/deb-temp

# fbc_deb_unpack $TMP_DIR/*.deb \
# 	--workdir $TMP_DIR


if fbc_pkg_cks_compare $TMP_DIR/*.deb $TMP_DIR/install ; then
	echo "Package did not change."
fi

dpkg --info $TMP_DIR/*.deb | grep -i 'Package:' | awk '{print $2}'
dpkg --info $TMP_DIR/*.deb | grep -i 'Architecture:' | awk '{print $2}'
dpkg --info $TMP_DIR/*.deb | grep -i 'Version:' | awk '{print $2}'
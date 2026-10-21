#!/bin/bash

DEB_PKG_NAME="tum-ics-cmake-tools"
DEB_PKG_VERSION="0.0.0"
DEB_PKG_MAINTAINER="Florian Bergner <florian.bergner@tum.de>"


DEB_PKG_DESCRIPTON=$(
cat <<-END
Description: CMake tools package
 This packet installs cmake tools.
 The package is created and maintained by:
 Florian Bergner <florian.bergner@tum.de>
 Emmanuel Dean <dean@tum.de>
END
)

# DEB_PKG_DESCRIPTON=$(cat $DEB_CONFIG_DIR/description)

# Default: Generic changelog
DEB_PKG_CHANGELOG=

# Default: BSD, PKG_MAINTAINER is author and maintainer 
DEB_PKG_COPYRIGHT=$(fbc_deb_copyright \
    --license "BSD" \
    --author-mail "florian.bergner@tum.de" \
    --maintainer "Florian Bergner <florian.bergner@tum.de>" \
    --maintainer "Emmanuel Dean <dean@tum.de>"
)


DEB_PKG_ARCH="all"

# Default: 'optional'
DEB_PKG_PRIORITY=

DEB_PKG_DEPENDS="libc6-dev cmake"

# optional
DEB_PKG_CONFLICTS=

# Default: No triggers
DEB_PKG_TRIGGERS=

# optional, can be empty
DEB_PKG_LICENSE="BSD"

# optional, can be empty
DEB_PKG_VENDOR="TUM ICS"

# optional, defaults to 'none'
DEB_PKG_CATEGORY=

# optional, defaults to 'https://www.ei.tum.de/ics/home/'
DEB_PKG_URL=

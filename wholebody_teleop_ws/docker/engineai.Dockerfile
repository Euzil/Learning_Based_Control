################################################################################
################################################################################
#
#       ros2 humble containter for devel
#
################################################################################
################################################################################

ARG ROS_DISTRO=humble
ARG UBUNTU_CODE_NAME=jammy

# default base image
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=${ROS_DISTRO}-desktop-full-${UBUNTU_CODE_NAME}

ARG N_THREADS=5

################################################################################
################################################################################
#
#       humble-devel-base
#
################################################################################
################################################################################

FROM ${BASE_IMAGE}:${BASE_TAG} AS humble-devel-base

LABEL maintainer="simon.armleder@tum.de"

ENV DEBIAN_FRONTEND=noninteractive

# USE German mirror to speed up things
RUN cp /etc/apt/sources.list /etc/apt/sources.list.old \
    && sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\// \
    http:\/\/de.archive\.ubuntu\.com\/ubuntu/' /etc/apt/sources.list

# devel tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    wget \
    ca-certificates \
    gnupg \
    software-properties-common \
    lsb-release \
    iproute2 \
    iputils-ping \
    dnsutils \
    host \
    net-tools \
    htop \
    bmon \
    rsync \
    vim \
    nano \
    git \
    ssh \
    wget \
    tar \
    curl \
    tree  \
    screen \
    && rm -rf /var/lib/apt/lists/*

# custom scripts
COPY setup/scripts/BashAddons /install/scripts/BashAddons
COPY setup/scripts/CMakeTools /software/CMakeTools
RUN cd /software/CMakeTools \
    && mkdir build-install \
    && cd build-install \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/install/usr \
    && make install

# ros devel tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglfw3 \
    libglfw3-dev  \
    libboost-all-dev \
    libboost-python-dev \
    python3-dev \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# python devel tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-tk \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# Keep numpy on 1.x: eigenpy/pinocchio and the ROS humble python packages are
# built against the numpy 1.x ABI and fail to import with numpy 2
# ("_ARRAY_API not found"). colcon needs setuptools<80.
# PIP_CONSTRAINT also guards later pip installs.
RUN printf "numpy<2\nsetuptools<80\n" > /etc/pip-constraints.txt \
    && python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir -c /etc/pip-constraints.txt "numpy<2"
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt

# ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joy-linux \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# default command
CMD [ "/bin/bash" ]

################################################################################
################################################################################
#
#           Stage: humble-sot-devel
#
################################################################################
################################################################################

FROM humble-devel-base AS humble-sot-devel

ARG N_THREADS

# pinned versions (same as h1_base_ws)
ARG QPMAD_VERSION=1.5.1
ARG EIGENPY_VERSION=v3.11.0
ARG COAL_VERSION=v3.0.2
ARG PINOCCHIO_VERSION=v3.7.0
ARG EIQUADPROG_VERSION=v1.3.1
ARG TSID_VERSION=v1.9.0
ARG NDCURVES_VERSION=v2.1.0
ARG CDDLIB_VERSION=0.94m
ARG MUJOCO_VERSION=3.3.3

ENV DEBIAN_FRONTEND=noninteractive

# make /opt/openrobots visible to cmake (c++ ros packages), the dynamic
# linker and python (ros python packages)
ENV OPENROBOTS_PREFIX=/opt/openrobots
ENV CMAKE_PREFIX_PATH=${OPENROBOTS_PREFIX}
ENV PKG_CONFIG_PATH=${OPENROBOTS_PREFIX}/lib/pkgconfig
ENV PATH=${OPENROBOTS_PREFIX}/bin:${PATH}
RUN echo "${OPENROBOTS_PREFIX}/lib" > /etc/ld.so.conf.d/openrobots.conf \
    && echo "${OPENROBOTS_PREFIX}/lib/python3.10/site-packages" > /usr/local/lib/python3.10/dist-packages/openrobots.pth

ARG SOT_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/opt/openrobots -DCMAKE_PREFIX_PATH=/opt/openrobots;/opt/ros/humble -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF"

RUN mkdir -p /software
WORKDIR /software

RUN git clone --branch ${QPMAD_VERSION} --depth 1 --recursive https://github.com/asherikov/qpmad.git \
    && cmake -S qpmad -B qpmad/build -DCMAKE_BUILD_TYPE=RELEASE \
    && cmake --build qpmad/build -j ${N_THREADS} --target install \
    && rm -rf qpmad

RUN git clone --branch ${EIGENPY_VERSION} --depth 1 --recursive https://github.com/stack-of-tasks/eigenpy.git \
    && cmake -S eigenpy -B eigenpy/build ${SOT_CMAKE_ARGS} \
    && cmake --build eigenpy/build -j ${N_THREADS} --target install \
    && rm -rf eigenpy

RUN git clone --branch ${COAL_VERSION} --depth 1 --recursive https://github.com/coal-library/coal.git \
    && cmake -S coal -B coal/build ${SOT_CMAKE_ARGS} -DBUILD_PYTHON_INTERFACE=ON -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=TRUE \
    && cmake --build coal/build -j ${N_THREADS} --target install \
    && rm -rf coal

RUN git clone --branch ${PINOCCHIO_VERSION} --depth 1 --recursive https://github.com/stack-of-tasks/pinocchio.git \
    && cmake -S pinocchio -B pinocchio/build ${SOT_CMAKE_ARGS} -DBUILD_PYTHON_INTERFACE=ON -DBUILD_WITH_COLLISION_SUPPORT=ON \
    && cmake --build pinocchio/build -j ${N_THREADS} --target install \
    && rm -rf pinocchio

RUN git clone --branch ${EIQUADPROG_VERSION} --depth 1 --recursive https://github.com/stack-of-tasks/eiquadprog.git \
    && cmake -S eiquadprog -B eiquadprog/build ${SOT_CMAKE_ARGS} \
    && cmake --build eiquadprog/build -j ${N_THREADS} --target install \
    && rm -rf eiquadprog

RUN git clone --branch ${TSID_VERSION} --depth 1 --recursive https://github.com/stack-of-tasks/tsid.git \
    && cmake -S tsid -B tsid/build ${SOT_CMAKE_ARGS} -DBUILD_PYTHON_INTERFACE=ON \
    && cmake --build tsid/build -j ${N_THREADS} --target install \
    && rm -rf tsid

RUN git clone --branch ${NDCURVES_VERSION} --depth 1 --recursive https://github.com/loco-3d/ndcurves.git \
    && cmake -S ndcurves -B ndcurves/build ${SOT_CMAKE_ARGS} -DBUILD_PYTHON_INTERFACE=ON \
    && cmake --build ndcurves/build -j ${N_THREADS} --target install \
    && rm -rf ndcurves

RUN wget -q https://github.com/cddlib/cddlib/releases/download/${CDDLIB_VERSION}/cddlib-${CDDLIB_VERSION}.tar.gz \
    && tar zxf cddlib-${CDDLIB_VERSION}.tar.gz \
    && cd cddlib-${CDDLIB_VERSION} \
    && ./configure \
    && make -j ${N_THREADS} \
    && make install \
    && cd .. && rm -rf cddlib-${CDDLIB_VERSION}*

RUN wget -q https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
    && tar zxf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
    && mkdir -p /opt/mujoco \
    && cp -r mujoco-${MUJOCO_VERSION}/. /opt/mujoco/ \
    && rm -rf mujoco-${MUJOCO_VERSION}*

RUN ldconfig \
    && python3 -c "import eigenpy, coal, hppfcl, pinocchio, tsid, ndcurves; print('pinocchio', pinocchio.__version__)"

WORKDIR /
RUN rm -r /software

CMD [ "bash" ]

################################################################################
################################################################################
#
#           Stage: engineai-devel
#
################################################################################
################################################################################

FROM humble-sot-devel AS engineai-devel

ENV DEBIAN_FRONTEND=noninteractive

ENV ONNXRUNTIME_VERSION="1.23.2"
ENV ONNXRUNTIME_FILENAME="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}"

ARG TORCH_VERSION=2.13.0
ARG TORCHVISION_VERSION=0.28.0
ARG ONNXRUNTIME_GPU_VERSION=1.23.2
# cuda 12.9: onnxruntime-gpu for python 3.10 stops at 1.23.x (cuda 12)
ARG TORCH_CUDA=cu129

RUN mkdir -p /software
WORKDIR /software

# engine ai specific libs
RUN git clone --depth 1 --recursive https://github.com/engineai-robotics/engineai_ros2_workspace.git \
    && cd engineai_ros2_workspace/src/third_party \
    && tar -xzf engineai_robotics_third_party_libs.tar.gz -C /opt \
    && cd /software && rm -rf engineai_ros2_workspace

# onnx runtime for c++ nn execution (cpu)
RUN wget -q https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_FILENAME}.tgz && \
    tar -xf ${ONNXRUNTIME_FILENAME}.tgz && \
    mkdir /opt/onnxruntime && \
    mv ${ONNXRUNTIME_FILENAME}/* /opt/onnxruntime && \
    rm -rf ${ONNXRUNTIME_FILENAME}* && \
    echo "/opt/onnxruntime/lib" >> /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# torch + onnxruntime-gpu for python nn execution
# Both use the CUDA 12 pip wheels (nvidia-*-cu12), so they share one set of
# cuda libs. The pip cuda lib dirs are registered with ldconfig so onnxruntime
# finds them without importing torch first.
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    --extra-index-url https://download.pytorch.org/whl/${TORCH_CUDA} \
    matplotlib \
    torch==${TORCH_VERSION}+${TORCH_CUDA} torchvision==${TORCHVISION_VERSION}+${TORCH_CUDA} \
    "onnxruntime-gpu[cuda,cudnn]==${ONNXRUNTIME_GPU_VERSION}" \
    onnx \
    && find /usr/local/lib/python3.10/dist-packages/nvidia -maxdepth 3 -type d -name lib \
        > /etc/ld.so.conf.d/nvidia-pip.conf \
    && ldconfig \
    && python3 -c "import onnxruntime as ort; assert 'CUDAExecutionProvider' in ort.get_available_providers(), ort.get_available_providers()" \
    && python3 -c "import numpy, pinocchio; assert numpy.__version__ < '2', numpy.__version__"

WORKDIR /
RUN rm -r /software

CMD [ "bash" ]

################################################################################
################################################################################
#
#           Stage: engineai-devel-vscode
#
################################################################################
################################################################################

ARG BASH_ALIASES=setup/.bash_aliases

FROM engineai-devel AS engineai-devel-vscode
ARG BASH_ALIASES

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        bash-completion \
	&& rm -rf /var/lib/apt/lists/*

# NOTE: The IDs do no matter here, the vs code build system will automatically
#   change them to the local user
# The user name also does not matter, it will be matched to the local one
ARG CONT_USER=devel
ARG CONT_UID=1000
ARG CONT_GID=1000

ENV CONT_USER=${CONT_USER}
ENV CONT_UID=${CONT_UID}
ENV CONT_GID=${CONT_GID}

ENV CONT_HOME_DIR=/home/${CONT_USER}

# Setup bash aliases
# COPY setup/.bash_aliases /etc/skel/.bash_aliases
COPY ${BASH_ALIASES} /etc/skel/.bash_aliases

# Install user
RUN deluser --remove-home ubuntu || true && \
    groupadd -g ${CONT_GID} ${CONT_USER} && \
    useradd -m -u ${CONT_UID} -g ${CONT_GID} -s /bin/bash ${CONT_USER} && \
    echo "${CONT_USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_${CONT_USER} && \
    chmod 0440 /etc/sudoers.d/99_${CONT_USER}

WORKDIR ${CONT_HOME_DIR}

# Custom commands
COPY setup/.bash_aliases /home/${CONT_USER}/.bash_aliases

# Enable bash completion for apt
RUN sed -i -e 's/^/#/' /etc/apt/apt.conf.d/docker-clean

# Enable color prompt
RUN sed -i '/force_color_prompt=yes/s/^#//g' ${CONT_HOME_DIR}/.bashrc

# default command
CMD [ "bash" ]

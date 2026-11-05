# Build context: repository root. Reuse the existing robotics base image.
ARG BASE_IMAGE=engineai-devel-vscode:latest
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10-venv libgl1-mesa-dri libglx-mesa0 mesa-utils \
    && rm -rf /var/lib/apt/lists/*
COPY deployment/quest/requirements-container.lock.txt /opt/quest/requirements.txt
RUN python3 -m venv --system-site-packages /opt/quest/venv \
    && /opt/quest/venv/bin/python -m pip install -r /opt/quest/requirements.txt \
    && source /opt/ros/humble/setup.bash \
    && /opt/quest/venv/bin/python -c 'import rclpy, mujoco, aiortc, aiohttp, onnxruntime, scipy; import numpy; assert numpy.__version__ == "1.26.4"'
COPY deployment/quest/container-entrypoint.sh /opt/quest/entrypoint.sh
ENV QUEST_VIDEO_PYTHON=/opt/quest/venv/bin/python \
    ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
WORKDIR /workspaces/Learning_Based_Control/wholebody_teleop_ws
ENTRYPOINT ["bash", "/opt/quest/entrypoint.sh"]
CMD ["run", "--wbc"]

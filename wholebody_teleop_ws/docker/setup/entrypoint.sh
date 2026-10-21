#!/bin/bash

# exit on error
set -e

# If the container is started as a non-root user (e.g., via --user),
# we cannot perform user creation logic. Just source and execute.
if [ "$(id -u)" -ne 0 ]; then
    exec "$@"
fi

# -----------------------------------------------------------------------------
# User Creation Logic
# -----------------------------------------------------------------------------
CONT_HOME_DIR=/home/${CONT_USER}

# 1. Create the Group if it doesn't exist
if ! getent group "${CONT_GID}" >/dev/null; then
    groupadd -g "${CONT_GID}" "${CONT_USER}"
fi

# 2. Create the User if it doesn't exist
if ! id -u "${CONT_USER}" >/dev/null 2>&1; then
    useradd -m -u "${CONT_UID}" -g "${CONT_GID}" -s /bin/bash "${CONT_USER}"
fi

# 3. ALWAYS ensure user is in hardware groups
# This is critical for the "Permission Denied" errors on /dev/nvmap
usermod -aG sudo,video,audio "${CONT_USER}"

# 4. Setup sudoers (Fixed path from /etc/apt/sudoers.d to /etc/sudoers.d)
if [ ! -d "/etc/sudoers.d" ]; then
    mkdir -p /etc/sudoers.d
fi
echo "${CONT_USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/99_${CONT_USER}"
chmod 0440 "/etc/sudoers.d/99_${CONT_USER}"

# 5. Sync Environment
if [ ! -d "${CONT_HOME_DIR}" ]; then mkdir -p "${CONT_HOME_DIR}"; fi
cp -r /etc/skel/. "${CONT_HOME_DIR}/" 2>/dev/null || true
chown "${CONT_UID}:${CONT_GID}" -R "${CONT_HOME_DIR}"

# enable bash completion for apt
if [ -f /etc/apt/apt.conf.d/docker-clean ]; then
    sed -i -e 's/^/#/' /etc/apt/apt.conf.d/docker-clean
fi

# Visual tweaks for the terminal
if [ -f "${CONT_HOME_DIR}/.bashrc" ]; then
    sed -i '/force_color_prompt=yes/s/^#//g' "${CONT_HOME_DIR}/.bashrc"
fi

# Check if the requested workspace directory exists
if [ ! -d "$CONT_PWD" ] ; then
    echo "WARNING: ENV CONT_PWD=$CONT_PWD not valid. Directory does not exist." 
    CONT_PWD=${CONT_HOME_DIR}
fi

# -----------------------------------------------------------------------------
# Execution
# -----------------------------------------------------------------------------

# Transform arguments to a string safely
cmd=''
for i in "$@"; do 
    cmd="$cmd ${i@Q}"
done

if [ -z "$cmd" ] || [ "$cmd" == " ''" ]; then
    cmd="/bin/bash"
fi

echo "--- Starting Container as User: ${CONT_USER} (UID: ${CONT_UID}, GID: ${CONT_GID}) ---"

# Use sudo -E to preserve the environment variables for CUDA/ROS
exec sudo -E -u "${CONT_USER}" bash -c "cd ${CONT_PWD} && $cmd"

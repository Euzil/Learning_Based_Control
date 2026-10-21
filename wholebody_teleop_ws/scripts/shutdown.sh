#!/bin/bash

# Ensure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "Error: 'sshpass' is not installed. Please install it first (e.g., sudo apt-get install sshpass)"
    exit 1
fi

# Define the configurations
declare -A HOST_CONFIGS
HOST_CONFIGS["nezha,REMOTE_HOST"]="192.168.0.163"
HOST_CONFIGS["nezha,REMOTE_USER"]="user"
HOST_CONFIGS["nezha,REMOTE_PASSWORD"]="1"
HOST_CONFIGS["nezha,REMOTE_PATH"]="/home/user/deployments/engineai_deploy_ws"

HOST_CONFIGS["orin,REMOTE_HOST"]="192.168.0.162"
HOST_CONFIGS["orin,REMOTE_USER"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PASSWORD"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PATH"]="/home/ubuntu/deployments/engineai_deploy_ws"

# Extract unique hostnames from the associative array keys
declare -A UNIQUE_HOSTS
for key in "${!HOST_CONFIGS[@]}"; do
    host="${key%,*}"
    UNIQUE_HOSTS[$host]=1
done

# Loop through each unique host and issue the shutdown command
for host in "${!UNIQUE_HOSTS[@]}"; do
    echo "----------------------------------------"
    echo "Initiating shutdown for: $host"
    
    REMOTE_HOST="${HOST_CONFIGS["$host,REMOTE_HOST"]}"
    REMOTE_USER="${HOST_CONFIGS["$host,REMOTE_USER"]}"
    REMOTE_PASSWORD="${HOST_CONFIGS["$host,REMOTE_PASSWORD"]}"
    
    echo "Connecting to $REMOTE_USER@$REMOTE_HOST..."

    # Execute the SSH command
    # -o StrictHostKeyChecking=no: Prevents the script from hanging on new/changed host keys
    # echo '$REMOTE_PASSWORD' | sudo -S: Passes the password to the remote sudo prompt
    sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER"@"$REMOTE_HOST" \
        "echo '$REMOTE_PASSWORD' | sudo -S shutdown now"
        
    # Check the exit status of the SSH command
    if [ $? -eq 0 ] || [ $? -eq 255 ]; then
        # 255 is often returned when the connection drops immediately due to the machine shutting down
        echo "Shutdown command successfully sent to $host."
    else
        echo "Failed to send shutdown command to $host. Please check credentials or connectivity."
    fi
done

echo "----------------------------------------"
echo "All shutdown signals sent."
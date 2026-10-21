# Docker

### Install
Install docker and nvidia docker toolkit

- https://docs.docker.com/engine/install/ubuntu/
- https://github.com/NVIDIA/nvidia-docker

- create folder and file `/etc/docker/daemon.json`
```json
{
    "default-runtime":"nvidia", 
    "ipv6": true,
    "fixed-cidr-v6": "fd00:dddd::/64",
    "runtimes": 
    {
        "nvidia": 
        {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
```

- restart the docker daemon
```bash
sudo systemctl restart docker
```

### Build

```bash
docker build -f engineai.Dockerfile \
  --tag 'engineai-devel-vscode:latest' \
  --build-arg ROS_DISTRO=humble \
  --build-arg UBUNTU_CODE_NAME=jammy .
```

# 服务器部署：VR 直连 + 浏览器监控

只保留一个部署方案：GitHub 拉取源码，Docker 运行接收器、WBC 仿真和相机，
使用服务器 NVIDIA Xorg 进行无显示器 GPU 渲染。电脑通过 SSH 查看相机监控。

```text
Quest ── TCP 8000 ──> 服务器 receiver ── 本机 UDP 9010 ──> 仿真
Quest <── WebRTC ──── 服务器视频服务（信令 TCP 8765，媒体由 ICE 协商）
电脑浏览器 <── SSH 转发 TCP 8765 ── 服务器 HTTP 相机监控
```

## 1. 服务器前提

- Linux x86_64，Docker Engine 与支持 `gpus` 的 Docker Compose（2.30+）。
- NVIDIA 驱动、NVIDIA Container Toolkit。
- 已配置无显示器 NVIDIA Xorg，具备可访问的 X11 socket 和 Xauthority cookie。
  不需要 GNOME/KDE 桌面；普通 SSH 登录本身不会创建这个 GPU 显示环境。
- Quest 可访问服务器的 TCP 8000、TCP 8765 和 WebRTC 媒体端点。
- 预留足够的镜像及构建空间，建议至少 30 GB。

安装参考：[Docker](https://docs.docker.com/engine/install/ubuntu/)、
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)。

GPU Xorg 的显卡 BusID、显示编号、授权路径取决于目标服务器，需要管理员配置。
部署前在服务器用实际 DISPLAY/XAUTHORITY 运行 `glxinfo -B`，确认
`OpenGL vendor string: NVIDIA`。不要使用 `xhost +` 开放显示权限。
本次未连接目标服务器，GPU 渲染和跨网络视频仍需部署后验收。

## 2. 拉取代码并构建环境

先将本次部署文件及相机监控修改提交并推送到 GitHub，然后在服务器：

```bash
git clone --branch v5 https://github.com/Euzil/Learning_Based_Control.git
cd Learning_Based_Control

docker build -f wholebody_teleop_ws/docker/engineai.Dockerfile \
  -t engineai-devel-vscode:latest wholebody_teleop_ws/docker

cd deployment/quest
cp .env.example .env
```

编辑 `.env` 中以下两项为服务器真实配置：

```dotenv
NVIDIA_DISPLAY=:0
NVIDIA_XAUTHORITY=/absolute/path/to/server-Xauthority
```

`NVIDIA_XAUTHORITY` 必须是存在且可读的授权文件，示例路径不是自动生成的。
然后：

```bash
docker compose build teleop
docker compose run --rm build
docker compose up -d receiver teleop
docker compose logs --tail=100 teleop receiver
```

第一步构建 Python/视频环境；第二步在独立 Docker 卷中编译 ROS 程序和机器人资源包；
第三步后台启动。不要复用其他主机的 build/install 目录。
原 robotics 基础 Dockerfile 下载外部工具链，本轮没有重新执行完整从零构建；
部署镜像和独立 ROS 构建此前在现有基础镜像上验证过。

不需要额外安装宿主机 `hts`、ROS 或 Unity。接收程序只依赖 Python 标准库；
仿真和视频使用容器内 Python 3.10 venv、NumPy 1.x，具体版本在依赖锁文件中。
GPU 渲染入口会检查 OpenGL 厂商，未正确使用 NVIDIA 时退出并提示，不会悄悄切到慢速软件渲染。
当前 WBC 策略仍使用 ONNX Runtime CPU provider；GPU 在这里用于画面渲染。

## 3. Quest 直连

Quest 应用选择 TCP Wireless，Server IP 填服务器地址，Port 为 8000。
Video Host 留空以跟随 Server IP，或填写同一可达服务器地址；视频端口为 8765。
修改 Unity 场景后必须重新构建并安装 APK 才能更新头显。

不需要在电脑启动 sockets.py，不需要 SSH 转发 8000，不需要 adb reverse。
能传动作不代表媒体端口已通：先用 Quest 浏览器检查
`http://服务器地址:8765/health`，再检查 `/preview` 的 WebRTC 播放。

## 4. 电脑浏览器查看

在电脑保持以下 SSH 连接：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18765:127.0.0.1:8765 USER@SERVER
```

打开：

- 相机监控：http://127.0.0.1:18765/monitor
- 相机状态：http://127.0.0.1:18765/health
- 单张截图：http://127.0.0.1:18765/snapshot.jpg

`/monitor` 使用 HTTP JPEG 快照，每秒最多刷新两次，只转发这个 TCP 端口即可。
旧画面超过 2 秒会变暗并提示，支持暂停刷新。它显示头部相机，不是可交互的 MuJoCo 全窗口。
`/preview` 是原来的 WebRTC 页面，其媒体不会被普通 SSH 端口转发自动带过来。
若电脑能直接访问服务器，也可以直接打开服务器的 `/monitor`，不必建立隧道。

## 5. 停止、更新和排错

```bash
# 以下在 deployment/quest 目录执行
curl http://127.0.0.1:8765/health
docker compose ps
docker compose logs --tail=100 teleop receiver
# 停止
docker compose down
```

SSH 断开不会停止后台容器。停止时不要加 `-v`，除非要删除编译卷。
更新代码时在仓库根目录 `git pull`；Python 改动后重启 teleop，
C++/ROS 改动后停止 teleop、重新 `docker compose run --rm build` 再启动。
依赖锁或 Dockerfile 改动需要重新 `docker compose build teleop` 并重建服务容器。

无画面时按顺序检查：NVIDIA OpenGL → 相机 health → 浏览器 monitor → Quest WebRTC。
本次已通过图像处理、HTTP 503、JPEG 缓存及浏览器暂停/恢复检查；目标服务器的实际 GPU 性能尚未验证。

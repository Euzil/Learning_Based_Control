# 服务器部署：VR 直连 + 浏览器监控

只保留一个部署方案：GitHub 拉取源码，Docker 运行接收器、WBC 仿真和相机，
使用服务器 NVIDIA Xorg 进行无显示器 GPU 渲染。电脑通过 SSH 查看相机监控。

如果账号没有 Docker 使用权限或 sudo 权限，可先按第 6、7 节建立用户目录下的
Python 虚拟环境，不需要 Conda。Python 环境不会解决 Docker 权限问题；
第 2 节的容器部署仍需管理员开通权限或提供服务器允许的容器方案。

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

Docker 接收服务不依赖宿主机 `hts`；开发、录制和模型实验使用第 6 节的独立虚拟环境。
服务器无需额外安装宿主机 ROS 或 Unity。接收程序只依赖 Python 标准库；
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

## 6. 独立 hts 虚拟环境（开发、数据处理、模型实验）

这是服务器宿主机上的普通 Python `venv` 环境，与 Docker 的 ROS/Python 环境独立。
使用 uv 在用户目录安装 Python 和创建虚拟环境，不需要 Conda，也不需要 sudo。
`requirements-hts.lock.txt` 保留当前 hts 中全部按包名安装的 pip 包及版本，
包括 hand-tracking-sdk、MuJoCo、ONNX Runtime、NumPy、SciPy、Matplotlib、Qt、Redis 等。
它不是只运行 sockets.py 的精简列表；也不是整台机器的系统库快照。
已在本机 Python 3.13 下以忽略现有安装的 pip dry-run 验证：46 个固定版本能够解析，
未实际创建新环境，也未更改现有 hts。
Qt 图形程序依赖显示服务，在纯 SSH 下安装成功不代表能弹出窗口。

以下命令在服务器的普通用户终端执行。首先安装用户级 uv：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
uv --version
```

参考：[uv 官方安装说明](https://docs.astral.sh/uv/getting-started/installation/)、
[Python 版本管理](https://docs.astral.sh/uv/guides/install-python/)。
下载 Python 和依赖需要服务器能访问对应下载站及包源。

首次创建 hts 环境并安装依赖（假设仓库在 `~/Learning_Based_Control`）：

```bash
uv python install 3.13
uv venv --python 3.13 --seed "$HOME/.venvs/hts"
source "$HOME/.venvs/hts/bin/activate"

cd ~/Learning_Based_Control/deployment/quest
python -m pip install -r requirements-hts.lock.txt
python -m pip check
python -c "import numpy, scipy, mujoco, onnxruntime, matplotlib; print(numpy.__version__, mujoco.__version__)"
cd ../../hand-tracking-streamer
python scripts/sockets.py --help
```

`--seed` 为虚拟环境安装 pip；Python 3.13 与原 hts 环境保持同一版本系列，
不会替换服务器系统 Python。若仓库路径不同，修改上面的 `cd` 路径。
若 `~/.venvs/hts` 已存在，先确认它是否是要使用的环境，不要覆盖；
需要新环境时，把创建和激活命令中的路径都改为 `~/.venvs/hts-server`。

以后每次登录只需激活，退出环境使用 `deactivate`：

```bash
source "$HOME/.venvs/hts/bin/activate"
# 完成工作后
# deactivate
```

不再需要执行 `conda env create` 或使用 `environment-hts.yml`；该 YAML 仅保留供已有 Conda 用户使用。
依赖仍来自 `requirements-hts.lock.txt`，不复制旧电脑绝对路径。
若某个版本不可下载，应核对原包源或导出对应 wheel，不应随意升级整套环境。
此文件不包含系统 NVIDIA 驱动或 Isaac Gym 本体；`isaacgym-stubs` 仅为接口 stub 包。

### 两个不能直接迁移的本地 editable 安装

- `pose`：原来指向 `TWIST2/pose`，该源码不在本次迁移目录中。使用相关功能前，
  先把对应源码迁到服务器，再在 hts 中 `python -m pip install -e /实际路径/TWIST2/pose`。
- `quest-wrist-tracker`：原来指向旧电脑 `/home/youran/hand-tracking-streamer`。
  当前接收/录制脚本可以直接从本仓库运行，不必执行 editable 安装。
  原项目 pyproject 声明 `numpy>=2.4.2`，实际 hts 却是 `numpy==2.2.6`；
  为保留实际环境，本快照不安装这个存在版本冲突的项目元数据。
  如要把它作为包安装，应先确认并修正项目依赖声明，不要使用 `--no-deps` 隐藏冲突。

要在 hts 中手动运行接收器时，先 `docker compose stop receiver`，避免占用同一个 8000 端口。
使用默认 Docker receiver 时，hts 可用于独立数据处理，无需启动第二份 sockets.py。

## 7. 模型训练环境

当前 hts 没有 PyTorch，恢复 hts 不等于已安装完整训练栈。
若训练本仓库 `ics_rl_lab`，使用它已有的 pyproject/uv.lock 创建独立 `.venv`：

第 6 节安装的 uv 可以直接创建训练环境，无需再向 hts 安装 uv 或 PyTorch。
在新终端执行（若当前激活了 hts，先运行 `deactivate`）：

```bash
export PATH="$HOME/.local/bin:$PATH"
cd ~/Learning_Based_Control/ics_rl_lab
uv sync --locked
uv run --locked python -c "import torch, mujoco; print(torch.__version__, mujoco.__version__); print('CUDA:', torch.cuda.is_available())"
uv run --locked scripts/train.py --help
```

训练依赖位于 `ics_rl_lab/.venv`，hts 依赖位于 `~/.venvs/hts`；两者独立。
`uv run` 自动使用训练项目环境，不需要手动激活 `.venv`。
ICS 项目要求 MuJoCo 3.6.0、Torch 2.10.0，以及配套 mjlab/mujoco-warp/warp-lang；
当前 hts 使用 MuJoCo 3.11.0，不能把两份依赖简单合并。
GPU 驱动兼容性、CUDA 可用性及实际训练需要在目标服务器验收；本次不启动训练。
其他训练项目也应按其依赖清单创建专用环境，不直接向 ROS 容器安装训练包。

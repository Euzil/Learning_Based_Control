# MuJoCo 头部画面接入 Quest

## 当前实现

链路：PM01 head_camera → `/simulation/head/image_raw`（ROS RGB8）→ Python WebRTC 服务 → Quest 现有 VideoPanelRenderer。

- 相机固定在 `head_yaw_link` 上，跟随模拟机器人的头部；它不是 MuJoCo 窗口中的观察者相机。
- 相机局部位置为 `0.10 0 0.10` 米，朝机器人 +X，图像向上对应 +Z，垂直视场角 60°。安装位置为初始近似，尚未按实物标定。
- 单目 1280×720，最多 30 fps；实际新帧率受 MuJoCo 主窗口渲染循环影响。WebRTC 仅发送新采集的图像，不通过重复帧补足 30 fps。
- 图像已经做 OpenGL 到 ROS 的上下翻转处理，时间戳为仿真时间。frame_id 为 `head_camera_optical_frame`，此阶段尚未发布相应 TF/CameraInfo。
- 使用现有 Quest WebSocket 信令协议，端口 8765。服务器的 SDP answer 内含 ICE candidates。
- Unity 在后台自动连接视频服务，收到已解码的视频帧后切换到头部跟随的单目大画面，隐藏透视层；视频停止或约 3 秒无新帧后恢复原视野。后台约每 3 秒重试，首次视频等待上限 15 秒。
- 这是单目视频。`run_quest_teleop.sh` 已接入头部 yaw，并在临时仿真模型中增加 pitch 来跟随抬头低头；普通 `run_quest_sim_video.sh` 不启动姿态控制。
- 当前场景主要是地面和天空，头部向前看时画面比较空，这是正常的。

## 已有容器中启动

在容器终端运行：

```bash
bash /workspaces/Learning_Based_Control/wholebody_teleop_ws/scripts/run_quest_sim_video.sh
```

脚本同时启动：

1. MuJoCo 模拟器（不启动行走策略）。
2. WebRTC 视频服务，TCP 8765 用于信令。

原有监听/控制脚本继续照常运行。本启动脚本默认不占用姿态端口，不会替换或中断原有追踪流程。若只做画面测试且没有运行原监听程序，可显式传入 `--telemetry-port 9000` 启用丢弃姿态的临时接收端。

按 Ctrl+C 结束。首次显示验证时，机器人保持初始姿态；无需连接游戏手柄。

脚本默认 ROS_DOMAIN_ID=87（若调用环境已指定则沿用），并设置 ROS_LOCALHOST_ONLY=1。ROS 通信在本机进行，Quest 通过局域网 WebRTC 接收画面。若单独运行其他 ROS 程序，需使用一致的 ROS 域。

## Unity / Quest 使用方式

1. 按原来的方式启动监听/控制脚本和 Unity 应用，开始原有 Streaming。Docker 视频服务没启动时保持原有视野和功能。
2. 需要机器人画面时运行上面的 Docker 启动命令。首个有效视频帧到达后自动切换，不需要打开 Video Stream 开关。
3. 关闭 Docker 中的视频程序后恢复原视野。视频连接错误不会调用 AppManager 的整体断开逻辑，手部、头部和控制器追踪仍继续。
4. 重启 Docker 视频程序后自动重连。

在 Unity 场景的 `VideoSystem → VideoStreamManager` 中：

- `Automatic Robot Vision`：默认启用。
- `Video Host`：空值时沿用原追踪的 Server IP。Unity 编辑器使用本机回环地址可连接同机 Docker。
- 如果 Quest 原来通过 USB/localhost 或广播传追踪数据，单独将 **Video Host** 设置为电脑的局域网 IP；不要改变原追踪地址。
- `Video Port`：8765。`Retry Seconds`：3。`Stalled Frame Timeout Seconds`：3。

Quest 与电脑需要网络互通。`adb reverse` 只转发 TCP，不能自动转发 WebRTC 的 UDP 媒体。保留原来的 USB 追踪也可以，但视频需要可达的 LAN 地址。

显示采用单目头部跟随面板，保留源画面比例，默认垂直视角 70°、距离 1.8 米，周围黑色。透视层原来的 hidden 状态和相机背景设置在退出机器人模式时恢复；不关闭追踪组件。可以在 `VideoPanelRenderer` 调整显示角度和距离。

在内层 Unity 项目 `hand_tracking_streamer` 中运行更新后的代码；头显上的旧 APK 不会自动更新，如果使用独立 APK，需要重新构建安装。

## 无 Quest 的浏览器预览

无需启动 Unity 或追踪监听脚本。在容器终端运行：

```bash
bash /workspaces/Learning_Based_Control/wholebody_teleop_ws/scripts/run_quest_sim_video.sh
```

在电脑浏览器打开 **http://127.0.0.1:8765/preview**。页面自动接收机器人头部视频，显示连接状态和帧率；“重新连接”按钮可重新建立连接。保持容器终端和模拟器窗口运行，Ctrl+C 停止。若已有旧版本视频服务运行，先停止并重新启动以加载新增页面。

该页面复用 Quest 的 WebRTC 视频服务，只显示画面，不发送追踪或关节指令。已用 Chrome 自动测试验证 1280×720 实际播放和手动重连，浏览器脚本无异常。此测试不验证 Quest 透视切换。默认场景仅有天空和棋盘地面。

## 检查

在电脑浏览器访问 `http://127.0.0.1:8765/health`，应看到 `source_ready: true`、width 1280、height 720。

不连接头显也可以验证完整的视频编解码链路。在另一个容器终端运行：

```bash
source /opt/ros/humble/setup.bash
/home/devel/quest-video-venv/bin/python \
  /workspaces/Learning_Based_Control/wholebody_teleop_ws/scripts/check_quest_video.py
```

测试会协商真实 WebRTC 连接并接收 30 帧，检查画面不是空白、时间戳递增。

当前验证：C++ 编译通过；RGB/BGR、行步长填充与非法图像的 3 项单元测试通过；MuJoCo 实际出图并通过 WebRTC 解码收到 1280×720 图像。最后一次 WebRTC 测试接收 30 个新帧耗时 1.73 秒（包含连接启动），新帧率有波动，未完成性能调优。启动日志出现过 `OpenGL error 0x501 in or before mjr_makeContext`，但随后实际出图和解码均成功；该图形环境告警尚未定位。没有通过 USB 连接的 Quest，因此头显显示、局域网延迟和 Android 解码还需要现场验证。

## 容器重建后的依赖恢复

本次已在当前容器创建 `/home/devel/quest-video-venv`，其依赖没有写入原有 Dockerfile。重建容器后，在容器里执行：

```bash
sudo apt-get update
sudo apt-get install -y python3.10-venv
python3 -m venv --system-site-packages /home/devel/quest-video-venv
/home/devel/quest-video-venv/bin/pip install \
  -r /workspaces/Learning_Based_Control/wholebody_teleop_ws/scripts/requirements-quest-video.txt
source /opt/ros/humble/setup.bash
source /workspaces/Learning_Based_Control/wholebody_teleop_ws/install/setup.bash
cd /workspaces/Learning_Based_Control/wholebody_teleop_ws
colcon build --packages-select mujoco_simulation pm01_description --symlink-install
```

`--system-site-packages` 让 Python 3.10 虚拟环境能访问 ROS Humble 的 rclpy；不要使用外层 HTS Python 3.13 环境启动 ROS 服务。

图像由 MuJoCo 的 OpenGL 上下文渲染，视频服务使用 aiortc 的编解码器，不要求安装 PyTorch/CUDA Toolkit。容器需有可用 DISPLAY 和 OpenGL 环境；当前版本不是无窗口 EGL 渲染器。

## 后续接入已有控制器

如果已有程序占用姿态端口，可仅启动视频服务，不启用测试姿态接收端：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
/home/devel/quest-video-venv/bin/python \
  /workspaces/Learning_Based_Control/wholebody_teleop_ws/scripts/quest_video_server.py
```

另外启动原有仿真与控制器，并确保 ROS 域一致。`--topic` 可切换 ROS RGB 图像来源；当前默认话题特意使用 `/simulation/` 前缀。此服务本身不控制机器人。

## 实现位置

- 相机位置：`src/robots/pm01_description/xml/pm01/pm01_cylinder_collision.xml`
- 图像分辨率和话题：`src/robots/pm01_description/configs/pm01.yaml` 中 `rgb_camera_sensors`
- ROS RGB 发布：`src/simulation/mujoco_simulation/src/robot_interface.cpp`
- WebRTC 服务：`scripts/quest_video_server.py`
- 一键启动：`scripts/run_quest_sim_video.sh`
- 连通测试：`scripts/check_quest_video.py`

API 参考：[MuJoCo 渲染接口](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)、[aiortc API](https://aiortc.readthedocs.io/en/latest/api.html)。

## 自动切换验证（2026-10-21）

Unity 6000.0.65f1 已编译更新后的脚本；`RobotVideoValidation.Run` 输出 `ROBOT_VIDEO_VALIDATION_PASSED`。检查覆盖首次帧等待、媒体停滞超时、重连计时重置、实际场景引用绑定，以及连续三次切换后的相机背景/清屏设置恢复和 16:9 显示比例。

编辑器测试完成后退出时发生 `tcache_thread_shutdown(): unaligned tcache chunk detected` 原生内存错误，进程退出码 134；因此不能将整次 Unity 运行记为正常退出。没有验证 Quest 真机透视切换、独立 APK 或真实网络断线恢复。缺失的 Meta SDK/Visual Scripting 包缓存已经重新下载，旧缓存和旧 Bee 编译缓存保留在 HTS 项目的 `work/package-cache-backup`。

视频自动连接需先按原流程启动 Streaming。若追踪通过 USB/localhost，记得为视频单独设置 Video Host。Docker 启动脚本现在默认不启动测试姿态监听端口。


## 视频依赖快速恢复

容器重建后若提示 `Video dependencies missing`，在容器的项目目录运行：

```bash
bash scripts/setup_quest_video.sh
```

该脚本安装 Python venv 支持、创建独立视频环境并验证 ROS/WebRTC 依赖；完成后重新执行 `bash scripts/run_quest_sim_video.sh`。

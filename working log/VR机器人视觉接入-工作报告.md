# VR 机器人视觉接入 MuJoCo 仿真工作报告

**报告日期：2026 年 10 月 21 日**
**项目：Hand Tracking Streamer / wholebody_teleop_ws**
**阶段：机器人视觉链路建设、Unity 自动切换及运行流程整理**

## 1、阶段成果

本阶段完成了 MuJoCo 中 PM01 机器人头部相机的彩色图像输出，以及经 ROS 2、WebRTC 到浏览器的视频链路。浏览器已实际接收并播放 1280×720 图像，无需连接 Quest 即可检查机器人视角。

Unity 端已加入“视频可用后切换机器人视野、断流后恢复原视野”的逻辑，并与原有追踪连接分离。相关脚本编译及编辑器检查通过，但 Quest 实机效果尚未验证；编辑器在自动检查完成后的退出阶段发生原生内存错误，不能将整次运行记为正常退出。

运行方式已整理为宿主机启动入口、容器内部启动入口和视频依赖恢复脚本。当前宿主机启动与 Ctrl+C 停止均已验证。容器挂载路径调整引起的 ROS 环境加载问题、容器重建导致的视频依赖丢失问题已处理。

**尚未完成的核心任务是 VR／手柄控制与 MuJoCo 的对接。** 当前启动脚本提供仿真与视觉，不启动原控制程序，也不代表已完成机器人遥操作闭环。SSH 远程查看的网络条件已分析，但无窗口渲染和仅经 SSH 隧道传输的视频模式尚未实现。

## 2、背景与目标

原项目在 Unity／Quest 端采集头部、手部和控制器数据，通过电脑端监听程序显示或处理追踪结果。用户描述原流程还能控制机器人行为，但本阶段未定位并验证其完整的机器人控制入口。

新的需求是在原有流程基础上使用 MuJoCo：佩戴者看到模拟机器人头部摄像头的画面，后续再利用 VR／手柄控制模拟机器人。用户要求原有监听脚本保留，Docker 程序未启动时原有体验不变，启动后自动进入机器人视觉模式。

据此，本阶段将“视觉接入”与“运动控制接入”分开实施，先建立可独立测试的视频链路，避免把视频连接失败等同于原追踪连接失败。

### 目标状态与当前完成度


| 目标                          | 当前状态             | 依据或剩余工作                            |
| ----------------------------- | -------------------- | ----------------------------------------- |
| MuJoCo 输出机器人头部彩色图像 | 已实现、已验证       | 实际取得 1280×720 RGB8 图像              |
| 无头显检查机器人视觉          | 已实现、已验证       | Chrome 播放及手动重连通过                 |
| Unity 自动切换与恢复视野      | 代码与编辑器检查完成 | Quest 实机和无线网络测试待完成            |
| 原监听脚本保留                | 已保留               | 新视频脚本默认不监听原姿态端口            |
| VR／手柄控制模拟机器人        | 待实现               | 需确认原控制入口、动作映射与 ROS 接口     |
| 无桌面主机上的仿真相机        | 待实现               | 当前依赖 DISPLAY 与 OpenGL 桌面上下文     |
| 只通过 SSH 隧道观看视频       | 待实现               | 当前 WebRTC UDP 媒体不随 TCP 隧道自动转发 |

## 3、系统结构与代码位置

### 3.1 当前视觉链路

```text
Docker 内
  MuJoCo PM01 机器人
      └─ head_yaw_link / head_camera
                  ↓ RGB8，1280×720
         /simulation/head/image_raw
                  ↓ ROS 2 订阅
         quest_video_server.py
                  ↓ WebRTC 视频 + WebSocket 信令
       ┌──────────┴──────────┐
       ↓                     ↓
电脑浏览器 /preview      Unity 视频接收模块
（已实测）              → Quest 视觉显示（待实机验证）
```

### 3.2 原追踪链路与后续控制链路

```text
原有链路保留：
Quest / Unity → 原监听与数据处理脚本 → 原可视化／控制流程

后续待接入：
VR／手柄输入 → 动作转换与控制器 → MuJoCo 关节／运动控制接口
```

视频服务本身不发布机器人关节命令。显示机器人画面与控制机器人是两项独立能力，不能通过视频正常就推断控制已经可用。

### 3.3 路径与运行环境

当前宿主机项目根目录：

```text
/media/youran/3C50A006509FC554/Users/12911/Documents/Study/TUM/Masterarbeit/Learning_Based_Control
```

对应容器挂载根目录：

```text
/workspaces/Learning_Based_Control
```


| 内容                        | 位置／设置                                                          |
| --------------------------- | ------------------------------------------------------------------- |
| Docker 容器                 | `wholebody_teleop_ws_devcont`                                       |
| 容器内 ROS 工作空间         | `/workspaces/Learning_Based_Control/wholebody_teleop_ws`            |
| Unity 工程                  | 宿主机项目根目录下的`hand-tracking-streamer/hand_tracking_streamer` |
| ROS 版本                    | Humble                                                              |
| 本次验证的 MuJoCo 版本      | 3.2.3                                                               |
| 本次使用的 Unity 编辑器     | 6000.0.65f1                                                         |
| 视频 Python 环境            | `/home/devel/quest-video-venv`，Python 3.10                         |
| 视频／信令服务端口          | 默认 TCP 8765；WebRTC 媒体另使用 ICE 协商的 UDP 连接                |
| 宿主机启动入口的默认 ROS 域 | 87                                                                  |
| ROS_LOCALHOST_ONLY          | 容器启动脚本设置为 1                                                |

旧路径 `/workspaces/wholebody_teleop_ws` 已不是当前挂载位置。宿主机启动脚本已支持自动查找新旧两个位置，也可通过 `ROBOT_WORKSPACE` 指定。

源码仍分属原来的项目，没有进行仓库合并或 Git 历史重写。原项目其他追踪脚本已有改动，不能将全部未提交差异归因于本次视觉工作。

## 4、已完成的技术工作

### 4.1 MuJoCo 头部虚拟相机

在机器人 `head_yaw_link` 上添加 `head_camera`。相机随模拟机器人的头部运动，与 MuJoCo 主窗口自由观察相机独立。

当前相机参数：


| 参数       | 值                            |
| ---------- | ----------------------------- |
| 局部位置   | `(0.10, 0, 0.10)` 米          |
| 朝向       | 沿机器人 +X 方向观察，+Z 向上 |
| 垂直视场角 | 60°                          |
| 输出尺寸   | 1280×720                     |
| 图像编码   | ROS`rgb8`                     |
| 图像话题   | `/simulation/head/image_raw`  |
| frame_id   | `head_camera_optical_frame`   |
| 时间戳     | 仿真时间                      |

安装位置是初始近似，尚未按实物相机标定，也尚未提供对应 CameraInfo 和 TF。RGB 渲染使用独立缓冲，保留现有深度相机功能。图像读取后做上下翻转，以转换 OpenGL 和 ROS 的图像行方向。

图像只在有订阅者时生成，并设置约 30 fps 的输出上限；主渲染循环和主机负载仍会限制实际新帧率，因此上限不代表稳定实测性能。

### 4.2 ROS 到 WebRTC 视频服务

新增 Python 服务订阅 ROS 图像，转换为 WebRTC 视频帧，复用现有 Quest 的信令格式。服务只保留最新图像，避免图像队列积压导致延迟不断增加；发送端不会用重复旧帧填补目标帧率。

提供以下接口：


| 接口                       | 功能                                           |
| -------------------------- | ---------------------------------------------- |
| `ws://主机:8765/`          | WebSocket 信令，交换 offer、answer、ICE 等消息 |
| `http://主机:8765/preview` | 浏览器视频预览                                 |
| `http://主机:8765/health`  | 检查图像源是否更新及图像尺寸                   |

服务还提供可选测试姿态接收端，但默认关闭。它只用于不运行原监听脚本时的显示测试，接收后丢弃姿态数据，不构成控制器。

### 4.3 Unity 视觉自动切换

自动切换依据是实际视频解码，而非 Docker 容器是否存在或信令是否连接。当前状态行为如下：


| 状态                             | 预期行为                         |
| -------------------------------- | -------------------------------- |
| 原 Streaming 未开始              | 不自动连接视频                   |
| 已开始 Streaming，视频服务未启动 | 原视野保持，后台尝试连接         |
| 信令已连接但尚未得到视频         | 保持原视野，等待首帧             |
| 有效视频帧到达                   | 隐藏透视显示，呈现机器人画面     |
| 视频断开或约 3 秒没有新帧        | 恢复原视野，保留追踪连接         |
| 视频服务重新可用                 | 后台重试并在收到有效帧后再次切换 |

Unity 视频采用头部跟随的单目大画面，保持视频比例，默认垂直显示角为 70°、距离为 1.8 米。进入机器人模式时保存透视层和相机背景状态，退出时恢复。此阶段不是双目立体渲染，也没有将佩戴者转头映射到机器人头部关节。

### 4.4 浏览器预览与启动管理

增加浏览器预览页，以便在没有 Quest 的情况下验证实际视频链路。预览页面自动连接视频，显示分辨率、连接状态和帧率，提供重连按钮。

启动管理分为三层：


| 脚本                             | 执行位置   | 作用                                        |
| -------------------------------- | ---------- | ------------------------------------------- |
| `start_robot_vision.sh`          | 宿主机终端 | 检查环境并通过 docker exec 调用容器启动入口 |
| `scripts/run_quest_sim_video.sh` | 容器内     | 启动模拟器和视频服务，退出时清理所启动进程  |
| `scripts/setup_quest_video.sh`   | 容器内     | 安装 venv 支持、创建视频环境并安装依赖      |

宿主机入口位于：

```text
/home/youran/Documents/Codex/2026-10-21/key/outputs/start_robot_vision.sh
```

## 5、详细操作流程

以下每一步都注明执行位置。本报告中的 IP 占位符应替换为实际地址；不要原样输入“主机IP”。

### 流程 A：首次准备或容器重建后恢复

**A1．宿主机终端：检查容器。**

```bash
docker ps --format '{{.Names}}'
```

预期包含 `wholebody_teleop_ws_devcont`。如果容器存在但已停止：

```bash
docker start wholebody_teleop_ws_devcont
```

**A2．宿主机终端：进入容器。**

```bash
docker exec -it -u devel wholebody_teleop_ws_devcont bash
```

**A3．容器终端：切换至当前工作空间。**

```bash
cd /workspaces/Learning_Based_Control/wholebody_teleop_ws
```

**A4．容器终端：恢复视频依赖。**

只有首次安装或环境丢失时需要执行：

```bash
bash scripts/setup_quest_video.sh
```

预期末尾显示 `Video dependencies ready.`。该步骤使用 Python 3.10 和 `--system-site-packages`，使视频虚拟环境能够访问 ROS Humble 的 Python 包。不要改用外层 HTS 的 Python 3.13 环境运行 ROS 视频服务。

本次环境中，初始下载曾因 IPv6 连接等待而延迟，使用仅对该次 pip 进程生效的 IPv4 下载方式后安装成功；没有修改系统网络配置。恢复脚本当前仍使用普通 pip 网络设置，不保证自动绕过同类网络问题。

**A5．容器终端：需要重新编译时执行。**

```bash
source /opt/ros/humble/setup.bash
./scripts/build.sh
source install/setup.bash
```

若已成功编译且没有修改 C++ 或相关配置安装内容，无需每次重编译。`./scripts/build.sh` 是子进程，编译完成不会自动更新当前终端环境，因此仍需 `source install/setup.bash`。

**A6．验证 ROS 包发现。**

```bash
ros2 pkg prefix pm01_description
```

预期输出当前工作空间的 `install/pm01_description` 路径。

### 流程 B：无需 Quest，在电脑上预览机器人视觉

**B1．宿主机终端：检查启动条件。**

```bash
bash /home/youran/Documents/Codex/2026-10-21/key/outputs/start_robot_vision.sh --check
```

预期输出 `Checks passed`。此命令不启动模拟器。

**B2．宿主机终端：启动仿真和视频。**

```bash
bash /home/youran/Documents/Codex/2026-10-21/key/outputs/start_robot_vision.sh
```

如果已经在容器终端，改用以下命令即可，两种入口任选其一，不要重复启动：

```bash
cd /workspaces/Learning_Based_Control/wholebody_teleop_ws
bash scripts/run_quest_sim_video.sh
```

预期 MuJoCo 窗口出现，终端打印视频服务启动信息。保留终端和模拟器窗口运行。当前程序需要有效桌面显示环境。

**B3．宿主机浏览器：查看视频。**

打开 [机器人头部视角预览](http://127.0.0.1:8765/preview)。无需启动 Unity、Quest 或追踪监听。

正常结果是页面显示 1280×720 视频，当前场景通常为天空和棋盘地面。旋转 MuJoCo 主窗口观察视角不会改变机器人头部相机的方向。

**B4．检查图像源状态。**

浏览器打开 [图像源健康检查](http://127.0.0.1:8765/health)，或在宿主机执行：

```bash
curl http://127.0.0.1:8765/health
```

预期 `source_ready` 为 `true`，宽高分别为 1280、720，`frames_received` 持续增加。

**B5．停止。**

在启动终端按 `Ctrl+C`。本次启动的模拟器和视频服务退出，Docker 容器本身继续运行。

### 流程 C：运行原有 VR 流程并测试自动视觉切换

此流程是下一轮 Quest 实机验收步骤，不代表已经在头显上完成验证。

**C1．保留并启动原有监听程序。**

按用户原有命令运行，不启动另一个占用相同姿态端口的接收程序。本阶段未确认原机器人控制入口，因此这里不提供猜测的控制启动命令。

**C2．打开正确的 Unity 工程。**

使用宿主机项目根目录下的：

```text
hand-tracking-streamer/hand_tracking_streamer
```

打开 `Assets/Scenes/Scene.unity`，检查 `VideoSystem` 上的引用绑定。运行原有流程并开始 Streaming；视频自动连接依赖该状态。

**C3．确认视频连接参数。**

在 `VideoSystem → VideoStreamManager` 中检查：


| 参数                          | 建议保留的当前值 |
| ----------------------------- | ---------------- |
| Automatic Robot Vision        | 开启             |
| Video Port                    | 8765             |
| Retry Seconds                 | 3                |
| First Frame Timeout Seconds   | 15               |
| Stalled Frame Timeout Seconds | 3                |

`Video Host` 留空时沿用原追踪 Server IP。若追踪采用 USB/localhost 或广播，必须单独填写视频主机的局域网 IP；不要因此修改原追踪配置。

`127.0.0.1` 在 Unity 编辑器中指电脑，在 Quest 中指头显自身。确认 Quest 与视频主机网络互通。

**C4．先不启动视频服务。**

观察原视野、手部／头部追踪和原有数据显示是否正常。该项用于验证新增逻辑没有影响原流程。

**C5．启动 Docker 视觉程序。**

按流程 B2 启动。预期在视频解码后切换机器人视觉；如果只有信令连接而没有画面，应继续保持原视野。

**C6．停止视频并验证恢复。**

在视频启动终端按 `Ctrl+C`。预期恢复原视野，原监听和追踪连接继续工作。检查手部、头部和控制器数据仍然更新。

**C7．重启服务并验证重连。**

再次执行启动命令。预期后台自动重连，并在收到有效帧后重新切换画面。

如果使用 Quest 独立 APK，应先将更新后的 Unity 工程重新构建安装；电脑源码变化不会自动更新已安装 APK。VR／手柄控制模拟机器人的功能需要后续另行接入，不能通过本流程直接获得。

### 流程 D：经 SSH 远程启动主机上的模块

**D1．远程电脑终端：连接主机并转发 TCP 端口。**

```bash
ssh -L 8765:127.0.0.1:8765 youran@主机IP
```

若远程电脑本地 8765 已被占用，可使用：

```bash
ssh -L 18765:127.0.0.1:8765 youran@主机IP
```

后一种方式的浏览器访问地址改为 `http://127.0.0.1:18765/preview`。

**D2．SSH 终端：进入容器。**

```bash
docker exec -it -u devel wholebody_teleop_ws_devcont bash
```

**D3．容器终端：检查显示环境并启动。**

```bash
echo "$DISPLAY"
cd /workspaces/Learning_Based_Control/wholebody_teleop_ws
bash scripts/run_quest_sim_video.sh
```

当前实现仍使用主机已有桌面会话。`DISPLAY` 非空只是必要线索，不能单凭它确认图形访问正常；还需要 X11/图形权限及 OpenGL 上下文可用。不要随意填写一个 DISPLAY 值来代替有效图形环境。

**D4．远程电脑浏览器：访问转发页面。**

打开 `http://127.0.0.1:8765/preview`，或使用 D1 中选择的本地端口。

必须区分两类情况：


| 网络情况                                           | 当前方案的边界                                                          |
| -------------------------------------------------- | ----------------------------------------------------------------------- |
| 远程电脑与主机可通过 LAN／VPN 直接交换 WebRTC 媒体 | 可能成功播放，需要实际检查 ICE／UDP 连通性                              |
| 只能通过 SSH 访问主机，其他流量不通                | 网页和信令可以通过 TCP 隧道，但 WebRTC UDP 媒体不自动转发，不能保证出图 |
| 主机没有桌面会话或可用显示上下文                   | 当前 MuJoCo 启动方式不满足，需要新增无窗口渲染                          |

“通过 SSH 启动”与“通过 SSH 隧道传输全部视频”是不同能力。当前尚未实现 HTTP/MJPEG 等完整走 TCP 隧道的替代预览模式，也未进行跨公网 SSH 视频验证。

## 6、验证与故障处理记录

### 6.1 已完成的验证


| 验证项               | 结果                       | 解释                                                                |
| -------------------- | -------------------------- | ------------------------------------------------------------------- |
| MuJoCo C++ 编译      | 通过                       | 相机及 ROS 发布代码编译成功                                         |
| ROS 图像输出         | 通过                       | 实际取得 1280×720 RGB8 图像                                        |
| 图像转换单元测试     | 3 项通过                   | 覆盖通道转换、行填充和非法输入                                      |
| WebRTC 接收测试      | 通过                       | 接收实际画面，时间戳递增且图像非空                                  |
| Chrome 浏览器播放    | 通过                       | 实际播放、重新连接，无页面脚本异常                                  |
| Unity 编译与自动检查 | 检查通过，但进程非正常退出 | 日志输出`ROBOT_VIDEO_VALIDATION_PASSED`，随后退出时发生原生内存错误 |
| 宿主机脚本启动／停止 | 通过                       | 启动后图像源更新，Ctrl+C 后测试进程与端口退出                       |
| 重建后视频依赖恢复   | 通过                       | 导入检查及三项单元测试再次通过                                      |
| Quest 实机自动切换   | 未验证                     | 仍需设备端验收                                                      |
| VR／手柄控制 MuJoCo  | 未完成                     | 原控制链路尚未接入                                                  |

一次 WebRTC 测试接收 30 个新帧约用时 1.73 秒，包含连接启动影响。该数字仅是单次观察，不是稳定帧率、端到端延迟或性能保证。

### 6.2 已处理的问题

**问题一：编译成功后仍提示 `pm01_description` 不存在。**
原因是当前终端未加载完整工作空间环境。已验证以下命令可恢复包查找，并使启动文件正常解析：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/Learning_Based_Control/wholebody_teleop_ws/install/setup.bash
ros2 launch engineai_deploy sim.launch.py --show-args
```

`--show-args` 用于解析验证，不实际启动机器人策略。普通 `sim.launch.py` 会启动项目原有策略等模块，它与本报告的“仅视觉启动脚本”并不相同。

**问题二：`Video dependencies missing`。**
容器重建后，原虚拟环境及 venv 支持缺失。已安装 `python3.10-venv`、重建 `/home/devel/quest-video-venv` 并安装依赖。新增恢复脚本和明确错误提示。依赖尚未固化进 Dockerfile，因此未来重建后仍可能需要恢复。

**问题三：Unity 缺少 Meta SDK／Visual Scripting 类型。**
检查发现依赖包缓存缺少文件。已备份并重新下载相关缓存，随后重新生成 Bee 编译缓存，最终完成脚本编译和自动检查。备份位于 HTS 项目的 `work/package-cache-backup`。

### 6.3 尚未定位的问题

- MuJoCo 启动日志出现过 `OpenGL error 0x501 in or before mjr_makeContext`；随后能够正常出图，但告警原因未定位。
- Unity 自动检查完成后退出时出现 `tcache_thread_shutdown(): unaligned tcache chunk detected`，退出码 134；不能据此宣称 Unity 的启动、运行、退出全流程均已通过。
- 实际渲染帧率存在波动，尚未进行系统性能优化或无线延迟测量。

## 七、下一阶段工作计划

1. **确认原控制入口。** 收集用户原来启动监听、动作转换和机器人控制所使用的具体命令，区分骨架显示、运动重定向与关节控制。
2. **设计保留监听脚本的数据接入方式。** 明确数据转发／订阅接口，避免两个程序竞争同一 TCP 端口，也不能假定两个 UDP 接收者会各自获得所有数据。
3. **接入模拟机器人控制。** 核对坐标系、关节顺序、控制模式及 ROS 域；启动模拟模块后才启用模拟器目标，明确是否还存在其他指令输出目标。
4. **完成 Quest 实机验收。** 按流程 C 验证无服务、首次连接、正常视频、断流、恢复及原追踪连续性。
5. **支持远程场景。** 根据实际需求增加可经 SSH TCP 隧道传输的预览模式，以及无桌面环境下的相机渲染。
6. **完善部署与维护。** 将视频依赖固化进镜像或容器创建流程，整理统一配置，补充性能测试和已知错误定位。

当前工作为后续闭环遥操作建立了可运行、可观察的视频基础；运动控制对接仍是独立且尚未完成的阶段。

## 附录 A：代码修改清单

以下路径分别以 `wholebody_teleop_ws` 和内层 Unity 工程 `hand_tracking_streamer` 为根目录。没有将所有仓库合并，也没有迁移原有源码位置。

### A.1 MuJoCo / ROS：wholebody_teleop_ws


| 文件                                                                                                        | 改动                                                                                                                          |
| ----------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `src/robots/pm01_description/xml/pm01/pm01_cylinder_collision.xml`                                          | 在`head_yaw_link` 添加 `head_camera`，跟随机器人头部                                                                          |
| `src/robots/pm01_description/configs/pm01.yaml`                                                             | 增加 RGB 相机配置，1280×720，话题`/simulation/head/image_raw`                                                                |
| `src/simulation/mujoco_simulation/include/mujoco_simulation/config_loader.h` 和 `src/config_loader.cpp`     | 增加并解析`rgb_camera_sensors` 配置                                                                                           |
| `src/simulation/mujoco_simulation/include/mujoco_simulation/robot_interface.h` 和 `src/robot_interface.cpp` | 增加独立 RGB 渲染缓冲、离屏渲染和 RGB8 图像发布；修正 OpenGL 图像上下方向；使用仿真时间戳；只在有订阅时渲染并限制最高输出频率 |
| `src/simulation/mujoco_simulation/src/sim_manager.cpp`                                                      | 让仿真循环响应 ROS 退出，并通知 UI 循环退出，支持启动脚本清理进程                                                             |
| `scripts/quest_video_server.py`                                                                             | ROS 图像转 WebRTC，兼容现有 Quest WebSocket 信令；仅保留最新帧，检测图像中断；提供`/health` 和 `/preview`                     |
| `scripts/quest_video_preview.html`                                                                          | 浏览器预览，自动连接、显示状态/帧率、断线重试和手动重连                                                                       |
| `scripts/run_quest_sim_video.sh`                                                                            | 统一启动 MuJoCo 与视频服务，退出时清理所启动进程；默认不启用临时姿态监听                                                      |
| `scripts/setup_quest_video.sh`                                                                              | 恢复 venv 支持和视频依赖，安装后执行导入检查                                                                                  |
| `scripts/requirements-quest-video.txt`                                                                      | 独立视频 Python 环境依赖                                                                                                      |
| `scripts/check_quest_video.py`                                                                              | 无头显 WebRTC 接收检查：接收实际图像、检查图像非空及时间戳递增                                                                |
| `scripts/tests/test_quest_video.py`                                                                         | RGB/BGR、行填充、非法图像转换测试                                                                                             |

相机初始局部位置为 `(0.10, 0, 0.10)` 米，向机器人 +X 看、+Z 向上，垂直视场角 60°。这只是初始安装近似，尚未标定；尚未发布 CameraInfo 或相机 TF。现有深度相机逻辑保留。

表格中两个 `src/*.cpp` 简写均指 `src/simulation/mujoco_simulation/src/` 下对应文件。

### A.2 Unity：hand_tracking_streamer


| 文件                                     | 改动                                                                                                 |
| ---------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `Assets/Scripts/AppManager.cs`           | 将可选视频连接与原追踪启动/断开流程分离，交由视频管理器后台处理                                      |
| `Assets/Scripts/VideoStreamManager.cs`   | 自动连接、首帧切换、超时回退、重连及会话隔离；视频失败不调用原追踪的整体断开逻辑                     |
| `Assets/Scripts/QuestVideoReceiver.cs`   | 使用接收视频的 transceiver；检查实际解码帧计数；关闭会话时结束待处理操作；阻止旧连接回调影响新会话   |
| `Assets/Scripts/VideoSignalingClient.cs` | 串行发送信令、发送超时、及时中断失效连接；按完整消息解码，限制消息大小                               |
| `Assets/Scripts/VideoPanelRenderer.cs`   | 将视频按源比例放置于头部跟随面板；进入机器人模式时隐藏透视层并设置黑背景；离开时恢复原透视和相机状态 |
| `Assets/Scripts/RobotVideoTiming.cs`     | 独立首帧等待与画面停滞计时逻辑                                                                       |
| `Assets/Scenes/Scene.unity`              | 保存自动切换开关、端口、重试与超时参数                                                               |
| `Assets/Editor/RobotVideoValidation.cs`  | 编辑器验证：场景绑定、计时策略、反复切换和原相机状态恢复                                             |

显示是单目头部跟随大画面，默认垂直显示角 70°、距离 1.8 米，保持视频比例。它不是双目立体渲染，也不会随佩戴者转头而驱动机器人头部。

原项目中已经存在其他手部、身体及控制器脚本改动，不能把整个 Git 工作区的差异都归到本次机器人视觉功能。

## 附录 B：常用入口速查


| 目的           | 入口                                                    |
| -------------- | ------------------------------------------------------- |
| 宿主机启动脚本 | 本报告同目录的`start_robot_vision.sh`                   |
| 容器内启动     | `bash scripts/run_quest_sim_video.sh`（先进入工作空间） |
| 容器依赖恢复   | `bash scripts/setup_quest_video.sh`（先进入工作空间）   |
| 浏览器视频     | http://127.0.0.1:8765/preview                           |
| 图像源状态     | http://127.0.0.1:8765/health                            |
| 操作与代码说明 | 本报告同目录的`机器人视觉-操作与代码说明.md`            |
| 详细使用说明   | 本报告同目录的`Quest-MuJoCo-使用说明.md`                |

本报告根据本次实际代码检查、执行日志与会话中已完成的验证整理。未将头显实测、远程 WebRTC 连通性或运动控制集成视为已完成成果。

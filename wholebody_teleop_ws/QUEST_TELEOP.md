# Quest 遥操作与策略选择

## 统一入口：保留固定基座、Walk、Run、WBC

在容器的 `wholebody_teleop_ws` 目录选择一个模式：

```bash
bash scripts/run_quest_teleop.sh          # 固定基座，原有 Quest 跟随
bash scripts/run_quest_teleop.sh --walk   # Walk + Quest 上肢角度覆盖
bash scripts/run_quest_teleop.sh --run    # Run + Quest 上肢角度覆盖
bash scripts/run_quest_teleop.sh --wbc    # WBC + Quest 参考输入
```

选择具体模型版本（当前统一接口支持 ONNX，不支持 JIT）：

```bash
bash scripts/run_quest_teleop.sh --walk --model models/pm01_amp_flat_walk/2026-09-27_22-30-26.onnx
bash scripts/run_quest_teleop.sh --run-model models/pm01_amp_flat_run/2026-09-24_11-03-04.onnx
bash scripts/run_quest_teleop.sh --wbc-model models/pm01_wbc/20260721_211647.onnx
```

`--walk-model`、`--run-model`、`--wbc-model` 都保留；它们等价于选择模式后使用 `--model`。
一次启动只能选择一个模式，换模型时 Ctrl+C 停止后重新启动。所有模式都保留追踪、标定配置加载和摄像头视频。
默认加载上述三个文件。模型关节顺序、历史长度、观测缩放、动作缩放和 PD 参数均读取各自元数据；
不兼容的模型在启动仿真之前报错。本地 20 个 ONNX 的元数据和输入输出形状已检查。

速度默认 `0 0 0`，可使用统一的 `--velocity VX VY YAW_RATE`，单位为 m/s、m/s、rad/s。
原 `--walk-velocity` 仍可使用，也提供 `--run-velocity` 和 `--wbc-velocity`。
`/simulation/quest/cmd_vel`（Twist）接口保留；首次使用后若 300 ms 无新消息则速度归零。
没有新鲜 Quest 数据时也会将速度归零。Run 并不保证零速度站立，与 Walk 一样，硬覆盖上肢可能导致漂移或倾倒。
WBC 会使用上肢参考参与推理，与 Walk/Run 的直接覆盖不同。
使用 `bash scripts/run_quest_teleop.sh --help` 查看入口参数。没有新增专用启动或测试脚本。


## 使用 WBC：沿用原有追踪、标定和视频

在开发容器的 `wholebody_teleop_ws` 目录启动：

```bash
bash scripts/run_quest_teleop.sh --wbc
```

需要沿用已有个人标定时：

```bash
bash scripts/run_quest_teleop.sh --wbc --body-profile configs/quest_calibrated.yaml
```

宿主机 `(hts)` 环境仍从 `hand-tracking-streamer` 目录运行原数据转发：

```bash
HTS_SIM_FORWARD_PORT=9010 python scripts/sockets.py --protocol tcp --host 0.0.0.0 --port 8000 --tally
```

停止旧仿真后启动以上命令，Quest 连接地址和视频流程保持原样。不需要单独运行测试文件，也不需要重编译 ROS。
缺少依赖时，在容器原 Python 环境运行：

```bash
/home/devel/quest-video-venv/bin/python -m pip install -r scripts/requirements-quest-teleop.txt
```

默认加载 `models/pm01_wbc/20260721_211647.onnx`。`--wbc-model PATH` 可选择其他兼容的 WBC ONNX。
不指定策略模式仍是固定基座模式。之前新增的三个专用 Walk 测试文件保持移除；运行接口均保留。

启动后的控制关系：

1. 场景不添加基座 weld，双腿和腰部可自由运动；保留物理关节限位。
2. WBC 以 50 Hz 运行，接收实际关节/IMU 反馈。默认平面速度、转向速度和 yaw/roll 指令为零，高度目标使用模型默认值，身体 pitch/roll/yaw 命令保持为零。
3. 未收到 Quest 时使用模型默认上肢参考，由策略生成全身关节目标。
4. 收到 Quest 后，双臂和头 yaw 的 11 个参考按模型训练范围限幅、限速，通过 `arm_delta` 进入策略的 872 维观测。
5. 原始 24 关节执行 `默认角度 + 策略动作 × 缩放 + 上肢参考偏移`，保留策略对上肢的修正。
   新增头 pitch 和双腕屈伸不在模型内，单独跟随；最终仅一个节点发布 27 关节命令。
6. Quest 超过 300 ms 无有效新帧时保持最后上肢参考，WBC 继续控制；机器人反馈失流或倾角超过 0.9 rad 时转为被动阻尼，需重置仿真。

日志 `Quest active: WBC references` 表示参考已接入。
WBC 接收 Quest 上肢参考并控制全身关节；不添加弯腰、降高或全身 IK 逻辑。
机器人引导标定仍在固定基座模式执行，随后用 `--body-profile` 加载。

运行文件只需关注：

| 文件 | 用途 |
|---|---|
| `scripts/run_quest_teleop.sh` | 统一启动入口 |
| `scripts/prepare_quest_teleop.py` | 生成固定基座或 WBC 浮动基座场景 |
| `scripts/quest_retarget.py` | 原有 Quest 重定向；WBC 模式只发布带名称的参考 |
| `scripts/quest_policy.py` | Walk/Run/WBC 推理、各自的参考处理及唯一的全身命令发布 |
| `scripts/quest_video_server.py` | 原有视频服务 |

验证范围：872 维观测与原有部署实现对照一致；无界面 10 秒默认站立、小幅抬臂各完成一次，
均未超过 0.9 rad 倾角或低于 0.45 m 基座高度，最大倾角约 0.151 rad，净位移约 0.12 m。
这是 Python MuJoCo 3.3.7 的短时结果，不能保证任意动作或长期稳定；实际 Quest、C++ 窗口与视频联动仍需现场确认。
若窗口报 `Authorization required` / `could not initialize GLFW`，需从有显示权限的开发容器终端启动。

## 固定基座模式

使用 TCP 8000 的 `sockets.py` 无可视化接收，通过本机 UDP 9010 转发完整 `Body joints_pose` 和 `Head pose`。控制器按 PM01 模型做双臂肘部/腕部位置 IK，并将头部朝向映射到 head_yaw。只消费最新帧，60 Hz 控制，关节限位和 1.5 rad/s 速度限制；300 ms 无有效姿态后保持最后关节目标。恢复追踪仍经过速度限制。

这是固定底座的上半身遥操作：底座通过仿真 weld 约束固定，腿保持默认弯曲姿态。没有行走、全身平衡或手指跟随。原始 PM01 只有头部 yaw、没有独立腕关节；当前控制原有 `elbow_yaw` 末端旋转关节，并在仿真中增加头部 pitch 和手掌屈伸（详见文末）。手臂方向以当前人体躯干为参考，按机器人臂长缩放；腰部保持默认位置。必须启用 Quest 上半身追踪并持续发送 70 个关节的 `Body joints_pose`。仅有手或控制器数据不会驱动此桥接器。

## 启动

1. 在宿主机 `(hts)` 环境，把原可视化程序停止后用以下命令重启（只增加环境变量）：

```bash
HTS_SIM_FORWARD_PORT=9010 python scripts/sockets.py --protocol tcp --host 0.0.0.0 --port 8000 --tally
```

Quest 原有 TCP 地址/端口保持不变。如果 TCP 断开后未自动恢复，在 Quest 上重新开始 Streaming。`record_visualize_body_stream.py` 也继承相同转发功能。未设置环境变量时完全沿用原接收行为。

2. 在已有开发容器中，进入 `wholebody_teleop_ws`，首次安装额外依赖：

```bash
/home/devel/quest-video-venv/bin/pip install -r scripts/requirements-quest-teleop.txt
```

3. 停止之前的 `run_quest_sim_video.sh`，启动含控制器的版本：

```bash
bash scripts/run_quest_teleop.sh
```

这个命令同时启动固定底座 MuJoCo、动作桥接器和原有 WebRTC 视频服务。沿用 ROS_DOMAIN_ID=87 与 ROS_LOCALHOST_ONLY=1。当前 Docker 是 host 网络，因此宿主机的 127.0.0.1:9010 可到达容器。桥接器只监听本机，不额外监听 Quest 的 TCP 8000。需使用其他转发端口时同步设置环境变量和启动脚本的 `--port`。

日志出现 `Tracking active: arms and head following` 表示收到有效身体数据。`Waiting for forwarded Body joints_pose` 表示尚未收到，`Tracking stale` 表示流已停滞。先尝试抬左臂、抬右臂、弯肘、左右转头。已有头部视频继续在 Quest 或 http://127.0.0.1:8765/preview 显示。

Ctrl+C 停止所有启动的子进程并删除临时模型。普通仿真配置没有被修改；新控制话题为 `/simulation/quest/joint_command`，状态为 `/simulation/quest/joint_state`，不向 `/hardware/joint_command` 发指令。

## 验证

容器里运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/home/devel/quest-video-venv/bin/python -m unittest discover -s scripts/tests -p test_quest_retarget.py
```

有可用 DISPLAY 且没有其他仿真/视频服务运行时，可运行独立集成检查（会弹出仿真窗口，发送合成身体姿态并自动退出）：

```bash
/home/devel/quest-video-venv/bin/python scripts/tests/check_quest_teleop_live.py
```

该检查验证真实仿真关节变化与失流保持。Quest 真人跟随的方向、延迟和舒适度仍需现场确认。

## 姿态映射修正（2026-10-22）

- Meta `Body_Head` 是骨骼坐标系，其局部 +Y 对应前向；此前误用 +Z 会让头部朝侧面并频繁触及限位。现在优先使用独立 `Head pose` 的相机 +Z；没有新鲜相机姿态时使用骨骼 +Y。俯仰接近垂直时保持 yaw，避免从接近零的水平投影计算跳变角度。
- 转发同时支持 `Body joints_pose`、`Head pose` 及带 `| f = ... | t = ...` 的调试头。重启宿主机 visualizer 才会加载这项更新，命令不变。
- 双臂采用躯干坐标下的上臂/前臂方向匹配，计算移动肩关节的相对 Jacobian，前臂端点延伸到机器人手端的近似位置。方向映射消除了人体绝对尺度的影响；手端位置仍是模型近似值，未做个人标定。
- 加入 80 ms 时间常数的目标平滑、直臂时的轻微默认姿态约束；降低上半身 PD 刚度并增加阻尼。只改跟随模式生成的配置。
- 测试覆盖骨骼与相机头坐标、左右转头方向、垂直视线保持、整体 yaw/尺度/平移不变性、弯肘、关节限位与速度限制；集成测试额外检查实际头关节角度和稳态摆幅。真人跟随仍需现场复核。

## 简单手部抬高补偿

默认 `--hand-height-bias 0.15`：在人体躯干坐标系中，先归一化前臂方向，再给向上分量增加 0.15，最后重新归一化。它不依赖世界位置或人体臂长，主要使水平/斜向前臂对应的机器人手端略微抬高；竖直垂臂不强行改变方向。这是经验补偿，不是完整的人体比例标定，也不是固定增加 15 cm。

停止并重启容器里的跟随脚本即可生效，宿主机 visualizer 不需要重启：

```bash
bash scripts/run_quest_teleop.sh
# 仍偏低时可尝试 0.25；设为 0 恢复无补偿版本。
bash scripts/run_quest_teleop.sh --hand-height-bias 0.25
```

范围 -0.5 到 0.5，正值抬高，负值降低；实际位移取决于当前姿态及关节限位。

## 容器重建后恢复完整环境

遇到 Python 路径不存在或 `No module named 'mujoco'`，在容器工作区执行 `bash scripts/setup_quest_teleop.sh`。该脚本恢复视频环境并安装遥操作依赖；仅执行视频安装脚本不会安装 MuJoCo Python 包。完成后重新启动 `bash scripts/run_quest_teleop.sh --hand-height-bias 0.25`。

## 个人臂长与比例标定（2026-10-23）

新增可选的 `--body-profile`。未指定时保持原有方向匹配；指定后使用相对机器人肩部的肘部、手端位置 IK。原方向模式每帧归一化，单纯在归一化前乘比例不会改变动作；新模式保留位置目标的长度，比例才真正参与 IK。

### 配置和启动

在容器的 `wholebody_teleop_ws` 中复制模板，然后编辑自己的尺寸：

```bash
cp configs/quest_body_profile.example.yaml configs/quest_body_profile.yaml
# 编辑 configs/quest_body_profile.yaml，再启动：
bash scripts/run_quest_teleop.sh --body-profile configs/quest_body_profile.yaml
```

模板中的 0.30/0.25 是示例，不是已测量的个人数据。所有长度和偏移单位均为 **米**。左右分别填写：

- `upper_arm_m`：肩关节中心到肘关节中心的长度。
- `forearm_m`：肘关节中心到腕部追踪点的长度，不含手指。
- `upper_gain`、`forearm_gain`：相应映射比例的微调倍率，默认 1，允许 0.5–1.5。建议先用实际长度，再小幅调整（例如 0.95/1.05）。改变目标长度会让 IK 在肘部和手端误差间折中，不会改变机器人的物理臂长。
- `hand_offset_m`：手端目标额外位移 `[前, 左, 上]`，坐标系为机器人躯干，单位米。例如 `[0, 0, 0.02]` 表示抬高手端目标 2 cm；右臂也使用相同轴向。
- `robot_hand_local_m`：机器人手端相对对应 `elbow_yaw_*_link` 的局部坐标。默认 `[0.03, 0, -0.14]` 沿用旧代码的近似点；它不是经过标定的腕部中心。模型当前没有明确的手端 site，不能声称仅凭 XML 已获得精确手端位置。只有确认模型参考点后才修改这个参数。
- `elbow_weight`、`hand_weight`：肘部与手端位置平方误差权重，默认 0.5/1。提高后者表示优先贴合手端，可能牺牲肘部方向。

个人模式默认 `hand_height_bias=0`，避免叠加旧的经验抬高补偿。仍可显式传入 `--hand-height-bias`；未使用个人配置时默认仍为 0.15。

仅校验个人配置、输出机器人参考长度和有效比例，不启动仿真或视频：

```bash
bash scripts/run_quest_teleop.sh --body-profile configs/quest_body_profile.yaml --check-profile
```

修改 YAML 后重启跟随脚本生效。Quest 和宿主机 visualizer 的数据协议不变，无需为此重编 APK。

### 映射定义与限制

机器人长度通过当前 MuJoCo 模型在默认姿态下计算：上臂为 `shoulder_roll` 到 `elbow_pitch`，前臂为 `elbow_pitch` 到配置的手端。人体肩→肘、肘→腕向量先转到人体躯干坐标，再按以下比例生成肩部相对目标：

```text
肘目标 = 人体上臂向量 × (机器人上臂参考长度 / 输入人体上臂长) × upper_gain
手目标 = 肘目标 + 人体前臂向量 × (机器人前臂参考长度 / 输入人体前臂长) × forearm_gain
         + hand_offset_m
```

身体整体平移、整体 yaw 不影响映射。个人模式要求追踪位置采用米制；追踪骨长与实际测量不符时，偏差也会进入目标，可通过个人参数/倍率校准。它不再对任意追踪尺度变化保持不变。默认姿态测得的是模型参考长度；机器人非共点关节轴及活动范围与人体不同，因此某些姿态存在不可消除的误差。

身高、肩宽暂不作为可填写的控制参数：肩、肘、腕已经来自 70 点身体追踪，映射以每侧肩为原点；这两个全身尺寸不会直接改善该相对手臂映射。腿部沿用固定底座逻辑；新增仿真手腕和头部俯仰跟随见文末。

保留 80 ms 目标平滑、关节限位和 1.5 rad/s 命令速度限制。每 5 秒输出左右肘/手端的 **命令姿态 IK 残差（cm）**，包含平滑后目标与限速后的命令姿态差异；它不是仿真实际关节反馈误差，也不是对真人精度的测量。目标不可达时仍会有残差，不能保证比例调节能让所有动作完全贴合。

建议先检查垂臂、前举、侧举和屈肘几种静态姿态，等动作稳定后再比较残差与画面。自动测试覆盖旧模式回归、个人比例对关节输出的影响、坐标变换、模型手端长度、非法输入、不可达目标和速度/限位；真实 Quest 的精度与延迟仍需现场测量。

## 机器人示范、多姿态采样标定

先退出旧跟随/视频脚本，保持 Quest 身体追踪和宿主机 UDP 转发运行。在容器工作区执行：

```bash
bash scripts/run_quest_teleop.sh --calibrate configs/quest_calibrated.yaml
```

无需预先创建输出文件。四个姿态依次为垂臂、前举、侧举和屈肘，全部参与拟合。机器人到位后准备 5 秒、采样 3 秒。**采样完成就保存并直接使用，不再安排第五个验证姿态，不检查真人是否模仿正确、不因抖动或拟合误差要求重采。** 采样区间内所有有效帧取平均，不剔除离群帧，也不要求至少 45 帧。追踪中断、非有限数或退化骨架仍需等待有效数据；机器人命令仍保留关节限位和速度限制。

输出个人配置与 `.report.yaml` 报告。报告中的拟合 RMS、每姿态误差仅供查看，不决定是否保存。自动采样配置使用 `fit_mode: sampled`，允许实际拟合得到的有限比例（包括零或负值）和偏移，不套用手工配置的经验范围；不会因为采样不标准而拒绝结果。动作完全不变、无法识别比例时使用零斜率和平均目标偏移，不强制重新采样。文件写入错误或无效数值仍会报告失败。

成功保存后自动开始真人跟随并启动机器人视频。标定期间保持 Quest 原视野/透视，通过电脑的 MuJoCo 全身窗口查看示范；没有新增 VR 第三人称示范。视频恢复需等待 Quest 自动重连，无需重编 APK。独立运行的旧视频服务需先自行退出。

已有文件需要替换时：

```bash
bash scripts/run_quest_teleop.sh --calibrate configs/quest_calibrated.yaml --overwrite-calibration
```

下次加载：

```bash
bash scripts/run_quest_teleop.sh --body-profile configs/quest_calibrated.yaml
```

固定站立肩高仍会保存：人体肩部相对标定参考的升降映射到机器人目标 Z，默认 `calibration.vertical_scale: 1.0`。这不是地面测量；Quest 重新居中后应重新采样。手端参考点仍是模型近似值。

代码回归检查（不属于用户标定中的验收步骤）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/home/devel/quest-video-venv/bin/python -m unittest discover -s scripts/tests -p 'test_quest*.py'
/home/devel/quest-video-venv/bin/python scripts/tests/check_quest_calibration_ros.py
```


## 原模型手腕活动与抬头低头

手腕现在直接控制原模型已有的 `elbow_yaw_l_joint` / `elbow_yaw_r_joint`，原有末端网格随它们转动。末端新增低饱和灰色圆润手掌与深灰连接件，用于可视化仿真屈腕。

- 每侧沿用模型的一个旋转自由度及原限位（约 ±150°），用于翻掌/前臂轴向旋转。另增加每侧 ±60° 的仿真屈伸关节；暂不增加侧偏和手指自由度。
- 使用 Body joints_pose 中 wrist（19/45）朝向，消除前臂摆动后提取绕前臂轴的旋转；方向按原关节轴确定。先设置腕旋转，再进行手臂 IK，使位置求解考虑原末端参考点随关节旋转的变化。
- 开始跟随时自然伸直手腕，首个有效姿态作为零位；完成四姿态采样后也重建零位，不增加验收步骤。丢失腕部朝向时保持该侧关节，保留平滑、1.5 rad/s 限速和原关节限位。
- 头部仍保留仿真新增的 pitch（±45°），抬头低头带动原头部外观和相机。原来的 24 个关节索引不变，末尾依次增加头部 pitch、左右腕屈伸，总共 27 个。

停止旧脚本后重新启动即可重新生成模型，无需重编 APK 或 C++：

```bash
bash scripts/run_quest_teleop.sh --body-profile configs/quest_calibrated.yaml
```

原始 PM01 XML 和硬件配置不变，运行时模型增加头部 pitch 和手掌屈伸；发布仍限于仿真话题。头部 pitch 和手掌屈伸不代表实机能力；轴向旋转继续使用原关节。新增手掌不参与碰撞，位置标定仍跟踪原腕部参考点。

`test_quest_orientation.py` 检查原末端转动、仿真屈腕跟随、丢失数据保持、屈伸限位、左右独立、坐标变换、头部相机方向及动力学；`check_quest_teleop_live.py` 检查 C++ 仿真中的原 wrist/elbow_yaw 和头部 pitch 反馈。

### 手腕固定角度偏差补偿

默认使用已确认的左手 +90°、右手 −90° 零位补偿，标定和正常跟随均自动生效，无需额外参数：

```bash
bash scripts/run_quest_teleop.sh --calibrate configs/quest_calibrated.yaml --overwrite-calibration
bash scripts/run_quest_teleop.sh --body-profile configs/quest_calibrated.yaml
```

需要调整时仍可用 `--wrist-offset-deg LEFT RIGHT` 覆盖默认值，单位为度，范围 ±150°。例如 `--wrist-offset-deg 0 0` 取消补偿。补偿不修改模型或个人位置标定文件。

第一帧的人手朝向对应补偿后的机器人零位，启动时保持期望的中立手势。补偿经过原有平滑、限速和限位；完成采样或重建零位不会叠加补偿。自定义参数不写入采样文件，后续启动需继续携带；默认的 `90 -90` 不需要显式传入。

屈伸使用 wrist 与食指、中指、小指近端指根的位置关系；首个有效掌面作为屈伸零位，完成采样后重新建立零位。启动时保持手腕自然伸直。原来的左右 `90 -90` 旋转补偿仍然默认生效。

## 手柄世界参考与 WBC 蹲起（2026-11-03）

新版发送端将摇杆累计位移、转向和输入是否仍有效单独发送给机器人。
在 `--wbc` 模式中，机器人根据实际里程计与世界目标的差生成身体坐标系速度，
由策略和腿部 PD 控制执行运动；不直接修改浮动基座位置。

- 左右摇杆横轴：只控制转向；两者同时操作时采用绝对值更大的横轴（相同时取右侧）。纵轴不控制移动或高度。
- 右食指扳机：前进；左食指扳机：后退。方向沿 Quest 参考朝向，启动时与机器人朝向对齐。
- 右侧握持扳机（中指侧键）：升高；左侧握持扳机：降低/下蹲。
- 同时按下相反方向的扳机时按力度相减；等力度抵消。超过死区后力度连续调节速度。
- 前进/后退及左右转向反向时，立即清除原方向尚未完成的目标。
- 松开移动扳机/转向摇杆：对应速度指令归零，世界目标重锚到当前位置/朝向，
  不继续追赶未完成目标；物理机器人仍有减速响应时间。
- 松开高度握持扳机：保留高度目标。WBC 高度指令以最多 0.2 m/s 变化，
  默认模型范围为 0.35–0.80 m；到限位后不累积超范围高度，反向操作立即生效。
  这不是实际高度精确跟踪保证。
- 新会话或超过 300 ms 的断流恢复：平移与转向重新锚定，不追赶断流期间累计目标。
- Quest 上肢参考或里程计过期：停止平面运动指令。
- Walk/Run 支持平面运动和转向，但不支持此高度指令；蹲起使用 WBC。

### 启动

**需要从已修改的 Unity 工程重新 Build And Run 到 Quest。** 旧 APK 没有独立摇杆报文，
只能继续使用原上肢跟随。Python/ROS 修改不需要重新编译 C++ 模拟器。

宿主机在 hand-tracking-streamer 目录：

```bash
python scripts/sockets.py --protocol tcp --host 0.0.0.0 --port 8000 --tally --forward-port 9010
```

开发容器在 wholebody_teleop_ws 目录：

```bash
bash scripts/run_quest_teleop.sh --wbc
# 使用已有个人标定时：
# bash scripts/run_quest_teleop.sh --wbc --body-profile configs/quest_calibrated.yaml
```

启动推流时面向预期的机器人正前方。收到新协议后日志显示
`Joystick world reference connected`。独立摇杆通道接管后，不再回退到旧的
`--velocity` 或 `/simulation/quest/cmd_vel` 指令，避免断流时恢复旧运动。
没有新协议时保留原速度接口行为。

### 数据与坐标

发送端 Body 数据包附加一条记录：

```text
Locomotion reference:, session, sequence, offset_x, offset_y, offset_z, yaw_rad, initial_heading_rad, move_magnitude, turn_input
```

`session` 为每次重置生成的 32 位十六进制会话 ID；序号单调增加。
位置单位为米，采用 Unity 右/上/前坐标。机器人转换为前/左/上，并按会话初始朝向
对齐。重复序号及已退役会话的迟到报文被丢弃。
平移目标相对当前位置的领先距离限制为 0.25 m，转向误差限制为 0.8 rad，
位置比例增益为 1.5；最终速度继续受模型元数据限制。

Body/Head 每帧头部增加 `| locomotion = x y z yaw_rad`，记录该帧已叠加的精确偏移。
新版发送端不再向头部/身体/手柄姿态叠加摇杆 yaw，元数据中的 yaw 为 0；
机器人解析时移除平移。接收端仍兼容旧版带 yaw 的元数据并撤销其旋转。
累计摇杆 yaw 只通过独立 Locomotion reference 通道控制机器人基座朝向。
真实转头继续控制头部关节；原可视化显示带平移偏移、但不带摇杆虚拟旋转的人体姿态。

ROS 接口：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/simulation/quest/locomotion_reference` | `std_msgs/String` | 独立摇杆参考原文 |
| `/simulation/quest/odom` | `nav_msgs/Odometry` | 模拟器实际基座位姿 |
| `/simulation/quest/world_target` | `geometry_msgs/PoseStamped` | 外环世界目标；z 是限幅后的高度目标 |

这里的 world_target 是可被松手、断流或领先距离限制重锚的操作目标，不是保持到达的导航路点。
当前实现适用于现有平地模拟场景，未增加障碍物规划或地形高度估计。

### 本次验证范围

- 45 项 Python 测试通过（含 8 项新增世界参考、松手、断流、会话与坐标测试）。
- 使用 Unity 6000.0.65f1 的 Android 编译参数编译 C# 脚本成功；有两条原有脚本警告。
  此检查没有生成或安装新 APK。
- Body/Head/Locomotion 三类报文的本地 UDP 转发通过。
- 隔离 ROS 域通信检查：收到 252 个世界目标和 256 个关节命令；
  检查到前进目标，松手后 XY 重锚、0.70 m 高度目标保留。
- 12 秒无窗口 MuJoCo + 实际 WBC 模型实验：2–4 秒推动前进，4 秒松开，
  6 秒后将高度目标降至 0.65 m。2 秒时基座 x=-0.099 m，4 秒时 x=0.115 m；
  4.02 秒速度指令为零。6 秒实际高度约 0.733 m，12 秒约 0.691 m。
  该实验完成 600 次策略更新，未触发 0.9 rad 倾角阈值。
  这些数值说明运动与高度通道有效，也表明实际高度尚未精确跟随目标；
  不等于验证整个允许高度范围或长时间稳定性。

尚未进行新版 APK 的 Quest 实机端到端验证；本次没有覆盖或安装现有 APK。

### 视频消失的检查与桌面授权

`--wbc` 启动脚本同时启动机器人视角视频，不需要另开视频模式。
若容器日志出现 `Authorization required` 或 `could not initialize GLFW`，
在宿主机桌面终端、项目根目录执行一次：

```bash
bash wholebody_teleop_ws/scripts/authorize_quest_display.sh
```

然后在容器内重新运行 `bash scripts/run_quest_teleop.sh --wbc`。
授权只复制当前显示会话的 X11 cookie；重新登录桌面或重建容器后可能需要重做。
在电脑打开 `http://127.0.0.1:8765/preview` 检查画面。
Quest 的 Video Host 必须使用电脑当前局域网 IP；2026-11-03 已从旧的
`10.181.253.197` 更新为 `10.181.229.242`。网络地址变化时需要再次更新。

这次修正还将右手柄输入从无效的 `SecondaryThumbstick + RTouch` 组合换成
明确的左右 RawAxis。新增反向移动与高度限位反向测试。

扳机改版验证：48 项测试通过，Unity Android 参数下 C# 编译通过。
实际 WBC + MuJoCo 12 秒测试中，2–4 秒向前，4–6 秒反向：4 秒基座
x=0.115 m，6 秒 x=-0.005 m；反向后的首次更新速度为负，6 秒释放后速度指令归零。
8 秒开始降低高度，12 秒实际高度约 0.701 m（目标 0.65 m），仍有模型跟踪误差。
视频授权修复后 `/health` 返回 source_ready=true，1280×720；WebRTC 在 1.50 秒内
收到 30 个新帧。测试用只显示模拟器和视频进程已关闭，避免与正式 WBC 启动争用端口。


### 可视化接收并转发手柄控制

在宿主机 `hand-tracking-streamer` 目录及 `(hts)` 环境运行：

```bash
HTS_SIM_FORWARD_PORT=9010 python scripts/visualizer.py --protocol tcp --host 0.0.0.0 --port 8000 --show-fingers
```

该入口保留人体、手指及手柄显示，并原样转发 Body、Head 和独立 Locomotion reference
到 UDP 9010，包括每帧的偏移元数据。与 `scripts/sockets.py` 二选一，不能同时占用 TCP 8000。
容器端继续运行 `bash scripts/run_quest_teleop.sh --wbc`。

控制映射与无可视化模式一致：左右摇杆横轴转向；右/左食指扳机分别前进/后退；
右/左握持扳机分别升高/下蹲。松开移动输入停止移动指令，高度保持。
发送端需已安装包含新按键映射的 APK；此 Python 修改只需重启 visualizer。


### 摇杆控制机器人整体转身

- 摇杆 yaw 仅进入机器人世界朝向外环，再成为策略的角速度指令。
- 不向头部、手臂或手柄追踪姿态叠加虚拟 yaw；真实头部转动继续独立跟随。
- 世界朝向误差限制由 0.4 rad 调整为 0.8 rad，外环最大输出从 0.6 提高到
  1.2 rad/s，之后仍按所选模型元数据限幅。默认 WBC 的转向范围为 ±1.2 rad/s。
- 固定基座模式无法整体转身。收到手柄报文但未启用策略时，会输出提示。
- 松开摇杆立即将角速度指令归零，实际身体需要减速；不直接修改机器人根部姿态。

此次涉及 Unity 的姿态发送逻辑，需重新 Build And Run；再重启宿主机接收程序和容器 WBC。

完整外环 + WBC 的最终验证：12 秒 MuJoCo 实验中，2–8 秒持续右转，
基座 yaw 从约 +9.4° 连续转过约 281°（角度跨越 ±180°），而头 yaw 关节保持在约 ±2.1° 内；
8.02 秒松杆后角速度指令为零，12 秒基座朝向约 +85.5°。
转身伴随约数十厘米位置漂移，不是精确原地旋转。整个实验未触发倾角保护。
49 项回归测试通过；随后调整转向上限并新增测试，12 项 locomotion 测试通过。
Unity C# 编译通过；尚需新版 APK 的设备端实测。

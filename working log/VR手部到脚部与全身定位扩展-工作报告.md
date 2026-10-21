# VR 手部定位扩展至脚部与全身数据采集工作报告

编制日期：2026-10-21。

**项目：Hand Tracking Streamer**

**阶段：1.脚部腿部关节扩展 , 2. 全身84个关节输出, 3.实时全身可视化检查, 4.全身数据录制**

依据：项目历史任务《扩展手部定位至脚部》、后续《用手柄重写软件》中的范围调整及当前代码。

## 1. 目标与成果概述

项目最初具备手腕位姿与每手 21 个关键点输出。扩展工作首先从 Meta Body 数据源读取左右腿/脚骨架，随后扩展为 84 个全身关节，形成网络接收、全身可视化、录制和离线回放流程。

历史中用户提供过 `Valid joints: 84/84` 的运行日志，并明确表示已有录制数据。这支持“曾取得有效的全身骨架输出”，但不等于独立测量了每个脚部关节的真实位置，也不等于机器人跟随已经完成。

后续需求主动收敛为上半身输入和手柄控制：当前 Body 流为 70 个关节，脚部流默认关闭，当前 visualizer 忽略 foot 标签。本文既保留早期扩展过程，也明确当前如何操作和如何复现实验。

## 2. 从手部到脚部的设计变化

原手部数据源提供 wrist 与手指 landmarks。脚部扩展没有把手指算法简单复制到脚趾，也没有新增足部相机识别算法，而是接入 `Oculus.Interaction.Body.Input.IBody`，读取 SDK 提供的下肢骨架。

每侧选择七个关节：


| 顺序 | SDK 名称后缀   | 用途           |
| ---- | -------------- | -------------- |
| 1    | LegUpper       | 大腿相关骨架点 |
| 2    | LegLower       | 小腿相关骨架点 |
| 3    | FootAnkleTwist | 踝部扭转骨架点 |
| 4    | FootAnkle      | 脚踝参考位姿   |
| 5    | FootSubtalar   | 足部骨架点     |
| 6    | FootTransverse | 足部骨架点     |
| 7    | FootBall       | 前脚掌骨架点   |

左右名称分别带 `Body_Left` 和 `Body_Right` 前缀。这是七个下肢/足部骨架节点，不是七个脚趾点，更不是每脚 21 个视觉测量点。历史需求和代码均涉及身体姿态推断，因此不要把 SDK 的有效性或置信度字段解释为足部测量精度保证。

## 3. 代码实现与数据结构

### 3.1 脚部独立流

`FootLandmarkStreamer.cs` 获取 `IBody`，检查数据有效性、骨架映射与 ankle 关节，读取脚踝位姿和七个骨架点，按原 TCP/UDP 配置发送：

```text
Left foot:, x, y, z, qx, qy, qz, qw
Left foot_landmarks:, [x, y, z] × 7
Right foot:, x, y, z, qx, qy, qz, qw
Right foot_landmarks:, [x, y, z] × 7
```

foot 是参考脚踝位姿；foot_landmarks 是以脚踝为原点、脚踝朝向为基准的局部坐标：

```text
p_local = inverse(q_ankle) × (p_joint - p_ankle)
p_world = p_ankle + q_ankle × p_local
```

单个节点读取失败时使用零坐标占位。零值因此不能直接认定为真实关节恰好在脚踝原点，后续精确数据集应增加逐关节有效性掩码。

### 3.2 全身统一流

随后新增 `BodyLandmarkStreamer.cs`，从同一 Body 源输出 root 与统一关节列表：


| 数据                      |   历史全身格式 |               当前上半身格式 |
| ------------------------- | -------------: | ---------------------------: |
| Body root                 |       7 浮点数 |                     7 浮点数 |
| Body joints_pose          |  84 × 7 = 588 |                70 × 7 = 490 |
| Body joints（关闭旋转时） |  84 × 3 = 252 |                70 × 3 = 210 |
| 关节范围                  | SDK 索引 0–83 | 0–69，至 RightHandLittleTip |

每个 pose 为 `x,y,z,qx,qy,qz,qw`。Body 关节位置是世界空间数据，不能再按 foot_landmarks 的脚踝局部公式处理。当前代码中的 `_upperBodyJoints` 明确列出 70 个节点。

### 3.3 接收、显示、录制


| 文件（相对项目根目录）                                                | 作用                                           |
| --------------------------------------------------------------------- | ---------------------------------------------- |
| `hand_tracking_streamer/Assets/Scripts/FootLandmarkStreamer.cs`       | 独立脚部数据与追踪诊断，目前默认关闭           |
| `hand_tracking_streamer/Assets/Scripts/BodyLandmarkStreamer.cs`       | Body 输出、缓存刷新和防重入处理                |
| `hand_tracking_streamer/Assets/Resources/OculusRuntimeSettings.asset` | Body 追踪配置，当前 joint set 序列化值为 1     |
| `hand_tracking_streamer/Assets/Scenes/Scene.unity`                    | 场景引用和启动权限请求；当前 Body 权限请求为 1 |
| `scripts/sockets.py`                                                  | 查看原始网络数据                               |
| `scripts/visualizer.py`                                               | 坐标转换与骨架显示，当前仅保留上半身线段       |
| `scripts/record_body_stream.py`                                       | 保存 Body 数据为 NPZ                           |
| `scripts/record_visualize_body_stream.py`                             | 单次接收同时录制和可视化                       |
| `scripts/replay_body_recording.py`                                    | 脱离 Quest 回放 NPZ                            |

## 4. 调试过程与问题修复

**组件挂载位置。** 历史中曾把外层/内层对象名字说反。正确原则是寻找含 `Body`/实现 `IBody` 的对象，而不是机械按“外层”“子对象”判断。用户实际层级为外层 OVRBody、内层 OVRBodyDataSource；streamer 应放在具有 Body 的对象上，Camera Rig Ref 配置在数据源中。

**脚部有效但不变化。** 历史日志曾同时出现 Connected、Valid、FullBodyFoot 为 True，但 ankle delta 为零。代码随后显式标记底层 source 和 Body 数据需要更新，避免持续读取缓存。诊断增加事件、帧数、瞬时和窗口位移等字段。静止时 delta 为零是正常现象，必须配合动作和更新事件判断。

**刷新递归。** 曾出现更新事件回调触发刷新、刷新又触发更新事件的递归。当前 `OnBodyUpdated()` 仅记录事件收到；`RefreshBodyData()` 使用重入保护及 finally 复位。这个修复保留在当前 Body streamer 中。

**重复组件与旧 APK。** 全身阶段统一为一个 BodyLandmarkStreamer，不再左右各挂一份。日志仍显示旧格式时先确认重新 Build And Run 的 APK 已运行，避免在旧程序上验证新代码。

**可视化误导。** 曾有 Root→Hips 连线看起来像第三条腿的问题，随后移除该辅助线；头部增加原始小点、立方体和方向轴，便于区分位置与朝向变化。当前进一步移除下肢绘制，这是后续范围调整，不是早期脚部功能从未存在。

## 操作环境与路径

本报告按 2026-10-21 可读取的代码整理。主机端项目根目录是：

```bash
cd "/media/youran/3C50A006509FC554/Users/12911/Documents/Study/TUM/Masterarbeit/Learning_Based_Control/hand-tracking-streamer"
```

Unity Hub 打开该目录内的 `hand_tracking_streamer`，然后打开 `Assets/Scenes/Scene.unity`。外层目录用于运行 Python 脚本，内层目录是 Unity 工程，不要混淆。旧对话中的 `/media/youran/SSD/...` 是旧挂载路径。

首次准备 Python 环境（已有可用环境时直接激活已有环境）：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy matplotlib
mkdir -p recordings
```

每次新开终端先进入项目根目录、激活环境。Matplotlib 可视化需要图形桌面；SSH 终端可先运行不依赖窗口的监听或录制脚本。这个数据采集实验不要求 Docker 或 MuJoCo 启动。

## 5. 早期脚部实验的 Unity 配置步骤

以下是开发流程复现说明；它不是当前正式版默认启用状态。本次文档编制没有改变场景或重新启用脚部。

1. 在独立实验副本或分支中打开 Scene.unity，保留原手部采集链路，以便对照。
2. 找到 Meta Body 数据源；若不存在，添加项目 SDK 中的 OVRBody prefab/对应 Body Tracking 对象。不要重复创建已有的数据源。
3. 查看实际 Inspector：找到含 `Body`/`IBody` 的对象，并确认其数据源引用有效。
4. 在 OVRBodyDataSource 的 Camera Rig Ref 中引用当前场景 Camera Rig；若组件展示 Hmd、Tracking Space，也分别连接场景已有引用。
5. 在 Meta/Oculus 配置中确认 FullBody joint set；确认运行时请求 Body Tracking 权限。当前配置文件保留相关设置，但仍需在设备上接受权限并核对实际追踪状态。
6. 为独立脚部实验添加两个 FootLandmarkStreamer，分别设 Left/Right。当前脚本新增了 `_streamFootData=false`，须在实验组件 Inspector 中显式开启 Stream Foot Data。
7. 先使用文本监听器验证 foot 包；当前 visualizer 明确忽略 foot 标签，不能仅开一个开关就恢复历史脚部画面。
8. 保存场景、编译、Build And Run 到 Quest，观察有效性及动作时坐标变化。
9. 若目标是统一全身流，则使用历史 84 关节版本的 BodyLandmarkStreamer，并只挂一个实例；通常关闭独立 foot 流以减少重复数据。

恢复 84 关节实验还需要匹配的 84 关节发送列表、下肢绘图连接和录制参数；仅将录制器改成 `--expected-joints 84` 不会让当前 70 关节发送器产生缺少的 14 个节点。不要覆盖当前正式版来做这项复现。

## 6. 当前版本可直接执行的采集步骤

### 步骤一：USB 与网络

```bash
adb devices
adb reverse tcp:8000 tcp:8000
adb reverse --list
python scripts/sockets.py --protocol tcp --host 0.0.0.0 --port 8000
```

Quest 使用 TCP Wired、127.0.0.1、8000，然后 Start Streaming。先确认出现 Body root 和 Body joints_pose。当前 joints_pose 应有 490 个数值；历史 84 关节 APK 应为 588。

确认完成后停止监听器。随后运行其他接收脚本前重新开始 Quest 推流；同一端口一次只运行一个接收入口。

### 步骤二：实时显示

```bash
python scripts/visualizer.py --protocol tcp --host 0.0.0.0 --port 8000 --show-fingers
```

检查左右手、头部朝向和上半身连续性。当前不显示腿脚是预期结果。历史脚部动作的验收需使用匹配的实验可视化实现。

### 步骤三：录制

停止 visualizer，使用组合入口：

```bash
python scripts/record_visualize_body_stream.py --protocol tcp --host 0.0.0.0 --port 8000 --expected-joints 70 --duration 20 --show-fingers -o recordings/body_upper_test.npz
```

若通过 SSH 且不需要图形窗口，使用：

```bash
python scripts/record_body_stream.py --protocol tcp --host 0.0.0.0 --port 8000 --expected-joints 70 --duration 20 -o recordings/body_upper_test.npz
```

这里的 USB reverse 应在实际连接 Quest 的电脑执行；如果 SSH 到另一台主机，必须另外明确网络转发拓扑，不能假定远程主机的 localhost 就是 Quest 所连电脑。

建议采集三组：静止站立、转头抬手、摇杆移动与高度变化；分别保存不同文件。历史全身实验另加入左右抬腿，但要通过骨架变化验证，不能仅用脚本正在发送作为抬腿正确的依据。

录制器时限按程序运行时间检查，提前启动后长时间不推流可能得不到有效帧。遇到准备时间不确定，可先不加 `--duration`，实际动作结束后 Ctrl+C 保存。

### 步骤四：检查文件

```bash
python - <<'CHECK'
import numpy as np
p = 'recordings/body_upper_test.npz'
d = np.load(p, allow_pickle=False)
for key in ('timestamps', 'root_poses', 'joint_poses', 'joint_count'):
    print(key, d[key].shape)
print('joint_count =', d['joint_count'])
print('finite =', np.isfinite(d['joint_poses']).all())
CHECK
```

当前期望 `joint_poses` 为 `[N,70,7]` 且 N>0，root 为 `[N,7]`。历史完整数据应为 `[N,84,7]`。数组维度正确只是格式检查，仍需查看动作是否连续、是否有零占位或冻结帧。

### 步骤五：离线回放

```bash
python scripts/replay_body_recording.py recordings/body_upper_test.npz --speed 0.5 --loop
```

回放不需要 Quest 或 Docker。可用 `--frame-step 3` 抽帧查看。当前播放器可加载符合形状要求的旧数据，但复用当前上半身骨架连接，不能保证完整重现历史下肢画面。

## 7. 84 关节旧数据兼容注意事项

若已经有产生 84 关节的历史发送器，纯录制可以显式指定：

```bash
python scripts/record_body_stream.py --protocol tcp --host 0.0.0.0 --port 8000 --expected-joints 84 --duration 20 -o recordings/body_full_84.npz
```

当前录制器对关节多于 expected 的帧会截取前 expected 个；因此默认 70 接收旧 84 流时会丢掉末尾下肢数据。反过来，expected=84 接收当前 70 流时会跳过帧。接收参数必须与发送端一致。

当前录制 metadata 中的 format 文本仍写着 70，即使传入 84 也可能产生描述不一致。检查时以 `joint_count` 和实际数组 shape 为准，做论文数据集前应修正该元信息。本次只记录问题，没有修改脚本。

## 8. 坐标与时间语义

Python 显示使用以下变换：

```text
x_display = z_unity
 y_display = -x_unity
z_display = y_unity
C = [[0,0,1],[-1,0,0],[0,1,0]]
R_display = C · R_unity · Cᵀ
```

这对应从 Unity 的右/上/前轴到显示空间的前/左/上轴。实际机器人接入还要做基座标定，不能只照文档中“翻转一个轴”的简化说法，也不要重复转换已经处理过的数据。

NPZ 的 timestamps 是电脑接收时基于 monotonic 的相对时间；不是硬件同步采样时间。录制器收到 joints_pose 后关联最近一条 root，不做严格相同帧号配对；各 streamer 也不是统一原子快照。单纯统计样本数量不能直接当作传感器真实刷新率或端到端时延。

`_frequency=0.01` 是 Unity 发送尝试间隔，不保证 100Hz 的独立骨架测量。关节位置/姿态可能包含推断、缓存重复或无效占位，需要单独做质量评价。

## 9. 验收和排查步骤


| 现象               | 优先检查                                              |
| ------------------ | ----------------------------------------------------- |
| No IBody component | streamer 是否挂在真实 Body 对象，而非仅有数据源的对象 |
| 无 Body 包         | 权限、Body 数据有效性、推流状态、IP/端口及 APK 版本   |
| 状态有效但坐标不变 | 实际做动作，检查更新事件、缓存刷新与窗口位移          |
| 闪退或重复刷日志   | 是否重复挂载；更新回调是否又触发同步刷新              |
| 有手但没有全身     | 发送标签与接收解析是否匹配、70/84 版本是否匹配        |
| 当前看不到脚       | 正式版默认关闭且 parser 忽略 foot，先确认实验目标     |
| NPZ 零帧           | 推流是否及时开始、是否先收到 root、expected 是否过大  |
| 图形窗口打不开     | 在图形桌面回放，SSH 先使用纯录制                      |

设备日志可用：

```bash
adb logcat -s Unity | rg 'BodyLandmarkStreamer|FootLandmarkStreamer|Exception'
```

应保存程序版本、设备与运行时版本、网络方式、采集时间、关节数和动作说明。历史的 HighConf=True、84/84 只能证明当时 SDK/脚本报告有效节点，不能替代与真实足部运动的误差对比。

## 10. 工作成果与后续衔接

本阶段把原手部采集扩展成了身体骨架数据管线，解决了组件接线、缓存刷新、刷新递归、全身可视化、头部方向表现以及录制回放等问题。当前保留这些基础设施，以 70 关节上半身模式服务后续控制系统。

后续机器人端可使用头部、腕部、肘部、躯干/root 等目标进行重定向；下肢由用户设计的机器人控制器生成。历史脚部实验及 84 关节数据可用于分析、对照和离线研究，不应自动作为当前机器人腿部控制输入。

本次只核对历史与代码并生成报告，未重新测量追踪精度或进行 Quest 联机试验。

## 11. 依据索引

- 历史任务《扩展手部定位至脚部》：`019f93b2-ba27-7571-ba9b-1df428e52bd4`。包括最初七关节脚部实现、用户提供的 84/84 日志、刷新递归修复、录制与回放请求。
- 历史任务《用手柄重写软件》：`01a08add-9670-7dc1-aa54-ad2b38b3bb25`。最后明确自行设计腿部，随后代码收敛为 70 个上半身关节。
- 当前代码：第 3 节文件列表、`CONNECTIONS.md`。旧路线文档曾描述完整遥操作路线，路线目标不能当作已经完成的闭环验收。

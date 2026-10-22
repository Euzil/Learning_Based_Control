# Quest 实时双臂与转头跟随

沿用 TCP 8000 的 `visualizer.py` 接收与显示，额外通过本机 UDP 9010 转发完整 `Body joints_pose`。控制器按 PM01 模型做双臂肘部/腕部位置 IK，并将头部朝向映射到 head_yaw。只消费最新帧，60 Hz 控制，关节限位和 1.5 rad/s 速度限制；300 ms 无有效姿态后保持最后关节目标。恢复追踪仍经过速度限制。

这是固定底座的上半身遥操作：底座通过仿真 weld 约束固定，腿保持默认弯曲姿态。没有行走、全身平衡、手指、腕部旋转或头部俯仰跟随；PM01 当前模型只有头部 yaw 关节。手臂方向以当前人体躯干为参考，按机器人臂长缩放；腰部保持默认位置。必须启用 Quest 上半身追踪并持续发送 70 个关节的 `Body joints_pose`。仅有手或控制器数据不会驱动此桥接器。

## 启动

1. 在宿主机 `(hts)` 环境，把原可视化程序停止后用以下命令重启（只增加环境变量）：

```bash
HTS_SIM_FORWARD_PORT=9010 python scripts/visualizer.py --protocol tcp --host 0.0.0.0 --port 8000 --show-fingers
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

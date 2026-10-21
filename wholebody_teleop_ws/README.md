# wholebody_teleop

### Install

Build the [docker image](docker/README.md)

### Simulation

Build:
```bash
./scripts/build.sh
```

Attach a gamepad in x mode to the computer

Run:
```bash
ros2 launch engineai_deploy sim.launch.py
```

### Real on robot

    1. Transfer code to the board 
```bash
./scripts/sync_src.sh nezha
```
    2. ssh into robot and build the ws
```bash
source ~/.venv/engineai_deploy/bin/activate
cd ~/source/engineai_deploy_ws
export ROS_DOMAIN_ID=69
./scripts/build.sh
```
    4. Press LB + RB to enable the motors
    5. Press LB + A to turn into pd-stand mode
    6. Press LB + CROSS_Y_LEFT to enter joint bridge mode
```bash
ros2 topic echo /motion/motion_state
```
    7. Run the Policy
```bash
python3 -m engineai_deploy.actor_critic_mlp --dry_run
```
    8. Press LB + A to stop the policy

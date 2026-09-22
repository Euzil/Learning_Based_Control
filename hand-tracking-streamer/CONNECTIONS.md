# Connecting and Streaming Data from HTS

This document describes how Hand Tracking Streamer (HTS) streams hand and wrist data, and how to connect to it over UDP and TCP.

## Controller palm pose format

`ControllerPalmPoseStreamer` can stream Quest Touch controller poses using the same wrist packet format as hand tracking. This is intended for robots that only need palm/wrist targets and do not have articulated fingers.

The in-headset input mode is fixed to the production mode:

```text
Full Hands + Controllers
```

`Full Hands + Controllers` streams hand-tracking wrist/finger landmarks, Quest Touch controller wrist poses, full-body skeleton data, and joystick locomotion data at the same time.

Controller palm data are streamed as UTF-8 CSV lines:

```text
Left wrist:, x, y, z, qx, qy, qz, qw
Right wrist:, x, y, z, qx, qy, qz, qw
Left controller_wrist:, x, y, z, qx, qy, qz, qw
Right controller_wrist:, x, y, z, qx, qy, qz, qw
```

- `Left wrist:` / `Right wrist:`: 7 floats -> hand-tracking wrist position and orientation, `x, y, z, qx, qy, qz, qw`.
- In `Full Hands + Controllers` mode, controller poses use `Left controller_wrist:` / `Right controller_wrist:` so they do not overwrite the real hand wrist poses.
- When `OVRCameraRig.trackingSpace` is found, controller local poses are transformed into Unity world space. Otherwise the raw tracking-space local pose is sent.

## Joystick locomotion

`JoystickLocomotionProvider` reads the Quest controller thumbsticks while streaming:

- Left thumbstick: moves the streamed body/foot skeleton on the ground plane, relative to the headset/camera facing direction.
- Right thumbstick X: updates an internal yaw offset for locomotion experiments.

The locomotion offset is applied to `Body root`, `Body joints_pose`, and streamed foot ankle positions before they are sent to the host. This lets the upper body come from VR/body/hand/controller tracking while the whole lower-body/root motion is driven by the controller joystick.

## Data format

HTS uses the OpenXR hand skeleton rig. Full details can be found in the [Meta documentation](https://developers.meta.com/horizon/documentation/unity/unity-handtracking-interactions/).


<img width="1999" height="951" alt="OXR Hand" src="https://github.com/user-attachments/assets/b964ca21-3cd6-44c7-b225-be503ed17a65" />


The full OpenXR hand skeleton has 29 joints, but many of them are static (palm, metacarpals, etc.) and are not streamed to save bandwidth. HTS sends the wrist plus 21 tracked joints per hand.

Tracked joint indices:

```text
[OpenXR index // streamed index: joint name]
1,  // 0: Wrist
2,  // 1: ThumbMetacarpal
3,  // 2: ThumbProximal
4,  // 3: ThumbDistal
5,  // 4: ThumbTip
7,  // 5: IndexProximal
8,  // 6: IndexIntermediate
9,  // 7: IndexDistal
10, // 8: IndexTip
12, // 9: MiddleProximal
13, // 10: MiddleIntermediate
14, // 11: MiddleDistal
15, // 12: MiddleTip
17, // 13: RingProximal
18, // 14: RingIntermediate
19, // 15: RingDistal
20, // 16: RingTip
22, // 17: LittleProximal
23, // 18: LittleIntermediate
24, // 19: LittleDistal
25  // 20: LittleTip
```

Tracked data are streamed as UTF‑8 CSV lines, with a leading label indicating side and type:

```text
Right wrist:, 0.2502, 1.0635, 0.2540, 0.194, -0.116, 0.094, -0.970
Right landmarks:, 0.0000, 0.0000, 0.0000, -0.0275, -0.0197, 0.0362, -0.0438, -0.0335, 0.0608, -0.0418, -0.0480, 0.0913, -0.0329, -0.0595, 0.1111, -0.0236, -0.0073, 0.0960, -0.0179, -0.0226, 0.1302, -0.0150, -0.0428, 0.1435, -0.0116, -0.0633, 0.1518, -0.0017, -0.0025, 0.0956, 0.0054, -0.0226, 0.1329, 0.0081, -0.0456, 0.1478, 0.0100, -0.0680, 0.1589, 0.0175, -0.0065, 0.0887, 0.0252, -0.0222, 0.1236, 0.0279, -0.0407, 0.1424, 0.0282, -0.0595, 0.1580, 0.0230, -0.0094, 0.0341, 0.0351, -0.0137, 0.0779, 0.0433, -0.0227, 0.1061, 0.0477, -0.0342, 0.1223
```

- `Right wrist:`: 7 floats → `x, y, z, qx, qy, qz, qw`.
- `Right landmarks:`: 63 floats → `[x, y, z] * 21` in the joint order listed above.

>[!IMPORTANT]
> Data streamed from HTS follows Unity's left-hand coordinate convention. For most applications, you will want to flip the incoming data's Y-axis for the right-hand coordinate convention.

### Experimental foot/body tracking format

`FootLandmarkStreamer` can stream foot and lower-leg joints from Meta body tracking when a scene GameObject provides an `Oculus.Interaction.Body.Input.IBody` source configured for the `FullBody` joint set.

Tracked body joint order per foot:

```text
[streamed index: body joint name]
0: LegUpper
1: LegLower
2: FootAnkleTwist
3: FootAnkle
4: FootSubtalar
5: FootTransverse
6: FootBall
```

Foot data are streamed as UTF-8 CSV lines:

```text
Left foot:, 0.2502, 0.1200, 0.2540, 0.194, -0.116, 0.094, -0.970
Left foot_landmarks:, 0.0000, 0.4100, 0.0200, 0.0000, 0.2100, 0.0100, 0.0000, 0.0200, 0.0000, 0.0000, 0.0000, 0.0000, 0.0100, -0.0300, 0.0300, 0.0200, -0.0500, 0.0700, 0.0300, -0.0700, 0.1100
```

- `Left foot:` / `Right foot:`: 7 floats -> ankle pose in world space, `x, y, z, qx, qy, qz, qw`.
- `Left foot_landmarks:` / `Right foot_landmarks:`: 21 floats -> `[x, y, z] * 7`, relative to the ankle pose, in the joint order listed above.

>[!NOTE]
> Meta body tracking does not provide hand-style toe landmarks. The foot stream exposes lower-body skeleton joints inferred by the headset/body tracking stack, so it is suitable for ankle/foot pose and coarse gait telemetry, not precise toe articulation.

### Full-body tracking format

`BodyLandmarkStreamer` streams the Meta `FullBody` skeleton from an `Oculus.Interaction.Body.Input.IBody` source. Add this component once to the GameObject that has the `Body` component.

Full-body data are streamed as UTF-8 CSV lines:

```text
Body root:, x, y, z, qx, qy, qz, qw
Body joints_pose:, [x, y, z, qx, qy, qz, qw] * 84
```

- `Body root:`: 7 floats -> body root pose in world space.
- `Body joints_pose:`: 588 floats -> 84 body joint world poses.
- Joint order is the SDK `BodyJointId` enum order from `Body_Root`/index `0` through `Body_RightFootBall`/index `83`.
- If rotations are disabled on the component, the second line becomes `Body joints:` with 252 floats -> `[x, y, z] * 84`.

## UDP connection

HTS can stream data via **wireless** UDP to a host on the same network as the Quest headset.

- Default target: `255.255.255.255:9000` (broadcast).
- You can change the target IP and port from the in‑game/network configuration menu.

>[!IMPORTANT] 
>Network Performance Notice UDP streaming depends heavily on your local network conditions. Specifically, Wi-Fi networks with high DTIM intervals may cause message batching (latency spikes) due to headset power-saving features (see Issue [#4](https://github.com/wengmister/hand-tracking-streamer/issues/4)).    
>For the more reliable performance, consider using the wired/wireless TCP connection.

## TCP connection - Wired

HTS can stream data via **wired** TCP using ADB. Connect your Quest to your machine with a data‑capable USB‑C cable.

- Default target: `localhost:8000` via ADB reverse loopback.

>[!NOTE]
> TCP streaming over USB generally provides the most consistent and reliable performance. In practice, HTS over TCP tops out at around 70 Hz Quest hand tracking frequency.

Set up the TCP reverse mapping before starting streaming from the HTS app:

```bash
adb reverse tcp:8000 tcp:8000
```

You can verify that the reverse rule is active with:

```bash
adb reverse --list
```

## TCP connection - Wireless

HTS can also stream data via **Wireless** TCP to a host on the same network as the Quest headset.

- Default target: `192.168.1.1:9000`
- You will need to identify your host's local IPv4 address for connection in HTS

Set up TCP server before starting streaming from the HTS app. Minimal example provided in `.\scripts\sockets.py`


## Troubleshooting

1. Have you enabled "Allow USB connection" on your Meta Quest?
   - Verify with `adb devices`. You should see your headset listed.
2. Make sure your firewall allows inbound traffic on the UDP/TCP port you are using.

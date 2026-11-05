# Quest 遥操作测试版 1

发布日期：2026-11-05。归档当前已构建的 APK，不重新构建或改变程序行为。

| 项目 | 值 |
|---|---|
| 发布标记 | 1.1.0-test.1 |
| APK 文件 | Quest-Control-1.1.0-test.1.apk |
| 应用显示名称 | hand-tracking-streamer |
| Android 内部版本 | 1.1.0 |
| versionCode | 7 |
| 包名 | com.wengmister.handtrackingstreamer |
| 架构 | arm64-v8a |
| 签名 | 当前 Android Debug 签名，v2 验证通过 |
| SHA-256 | e0feb830fecfa1c67f363b567bb0f2b0151edb366f0a3ea14912c4badd003335 |

`test.1` 是此次归档的测试版标记；APK 内部名称和版本仍为上表所列。
不必为了让应用留在头显里而重建 APK。Unity Build And Run 安装过的应用通常已经留在设备上。

## 安装一次，之后直接打开

在自己的电脑上操作，不是在远程仿真服务器上操作。
头显启用开发者模式，用 USB 连接电脑，并在头显内允许 USB 调试。
在此目录运行：

```bash
adb devices -l
sha256sum -c SHA256SUMS
adb install -r Quest-Control-1.1.0-test.1.apk
adb shell pm path com.wengmister.handtrackingstreamer
```

只有设备状态为 `device` 才能安装；`unauthorized` 时需在头显允许调试。
多设备时在每条 adb 命令后加 `-s 设备序列号` 明确选择头显。
`-r` 为同签名覆盖安装，保留应用数据。若提示签名不兼容或版本降级，
先检查安装版本和签名，不要直接卸载以免丢失设置。

安装成功后可拔掉 USB，从 Quest 应用库的“未知来源 / Unknown Sources”找到
`hand-tracking-streamer` 启动。关机重启后也不需要重新安装。
通过 Wi-Fi 传动作时仍需电脑/服务器接收程序运行；保存到头显不等于仿真也在头显运行。

官方说明：[ADB 安装](https://developers.meta.com/vr/documentation/native/android/ts-adb/)、
[从应用库打开安装的应用](https://developers.meta.com/vr/essentials/quick-start/)。

## 当前测试版边界

冻结的是现有 APK，未在此发布步骤重新进行头显现场验收。
APK 场景资源中检测到的局域网地址：10.181.230.54。
当前 Unity 场景的 Video Host 固定为 `10.181.230.54`；此版用于当前电脑网络测试。
换服务器时需确认视频目标地址，追踪 Server IP 与固定 Video Host 不一定同步。
如需修改视频地址或程序功能，才需要重建并重新安装；平时启动无需 Unity。

当前沿用调试签名作为内部测试版，不包含商店发布流程。
未来更新保持包名和签名一致，并递增 versionCode。保留现有签名密钥的安全备份，
不要把私钥或 keystore 提交到 Git。仅重命名 APK 文件不会改变内部版本。

## 文件管理

此目录 APK 是冻结副本；后续 Unity 构建可以继续更新原 `hand_tracking_streamer.apk`。
归档 APK 被 `.gitignore` 排除以避免在 Git 中再加入一份大二进制文件；
说明和 SHA256SUMS 可正常提交。需要分发时直接传 APK 文件，无需 Unity 工程。

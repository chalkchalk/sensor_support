# WTRTK-982DT 定位、双天线定向与 NTRIP（ROS 2 Jazzy）

这是一个可独立放入 ROS 2 工作空间 `src` 目录的单一功能包，适用于 Ubuntu 24.04 +
ROS 2 Jazzy。包内同时包含 WTRTK-982DT 自定义定向消息、Python 节点、launch、配置
与测试，不再依赖另一个接口包。

## 实现的功能

- 读取 GNSS 串口输出，并稳健地按行拆分 NMEA 数据；
- 启动时向 WTRTK-982DT 发送 `GPGGA <周期>` 和 `GPHPR <周期>`；
- 将 `$GPGGA` / `$GNGGA` 转成标准 `sensor_msgs/msg/NavSatFix`；
- 兼容 `$GPHPR`、`$GNHPR` 等 HPR talker 前缀，发布航向、俯仰、横滚、解状态等；
- 输出接收机原始真北航向、安装偏角修正后的车辆航向和 ROS ENU yaw；
- 取得 GGA 后可选连接 NTRIP caster；
- 使用 Basic Auth 请求挂载点，周期性上传最新 GGA；
- 接收 RTCM 二进制流并原样写回 GNSS 串口；
- 串口或 NTRIP 断开后自动重连，退出时关闭资源；
- HPR 超时后明确发布一次 `data_valid=false`，避免下游继续使用旧航向。

## 依赖与构建

```bash
sudo apt update
sudo apt install python3-serial python3-colcon-common-extensions
cd /path/to/your_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select wtrtk_982dt_driver
source install/setup.bash
```

如果当前用户没有串口权限，推荐加入 `dialout` 组，然后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## 配置与运行

编辑 `src/wtrtk_982dt_driver/config/gps_ntrip.yaml` 设置串口、caster 和挂载点。为避免把
密码写进代码仓库，账号和密码默认从环境变量读取：

```bash
export NTRIP_USERNAME='你的账号'
read -rsp 'NTRIP password: ' NTRIP_PASSWORD && export NTRIP_PASSWORD && echo
source /opt/ros/jazzy/setup.bash
source /path/to/your_ros2_ws/install/setup.bash
ros2 launch wtrtk_982dt_driver gps_ntrip.launch.py
```

默认配置会启用 NTRIP，并自动使用中国大陆 `ml` 配置：

```yaml
ntrip_enabled: true
ntrip_profile: ml
```

将 `ntrip_enabled` 设为 `false` 可关闭差分；将 `ntrip_profile` 改为 `hk` 可切换到
中国香港配置。上述行为都由 `gps_ntrip.yaml` 决定，正常启动无需再传 launch 参数。

中国大陆使用指令（对应 gps_ntrip_ml.yaml）：

```bash
ros2 launch wtrtk_982dt_driver gps_ntrip.launch.py \
  config_file:=/absolute/path/to/gps_ntrip_ml.yaml
```

中国香港使用指令（对应 gps_ntrip_hk.yaml）：

```bash
ros2 launch wtrtk_982dt_driver gps_ntrip.launch.py \
  config_file:=/absolute/path/to/gps_ntrip_hk.yaml
```

## 话题设计

节点只保留四个职责明确的话题：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/msg/NavSatFix` | 经纬度、高程及定位是否有效 |
| `/gnss/attitude` | `wtrtk_982dt_driver/msg/GnssAttitude` | GPHPR 定向结果及质量信息 |
| `/gnss/nmea` | `std_msgs/msg/String` | 接收机全部原始 `$...` 语句，供诊断/记录 |
| `/gnss/device_connected` | `std_msgs/msg/Bool` | 串口设备当前是否在线 |
| `/gnss/fix_valid` | `std_msgs/msg/Bool` | 当前定位解是否有效 |
| `/ntrip/connected` | `std_msgs/msg/Bool` | NTRIP 数据链是否已连接 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 设备、定位、定向、新鲜度和解析错误诊断 |

旧的 `/nmea`、`/gga`、`/fix` 和 `/ntrip_connected` 不再发布。GGA 仍在节点内部用于
生成 `/gnss/fix` 和上传给 NTRIP caster，不再单独重复发布。

常用检查命令：

```bash
ros2 interface show wtrtk_982dt_driver/msg/GnssAttitude
ros2 topic echo /gnss/fix
ros2 topic echo /gnss/attitude
ros2 topic echo /gnss/device_connected
ros2 topic echo /gnss/fix_valid
ros2 topic echo /ntrip/connected
ros2 topic echo /diagnostics
ros2 topic hz /gnss/fix
ros2 topic hz /gnss/attitude
```

`/gnss/attitude` 中的主要字段：

- `raw_heading_deg`：接收机原始航向，真北为 0°、顺时针增加；
- `heading_deg`：加上 `heading_offset_deg` 后的车辆航向，范围 `[0, 360)`；
- `yaw_enu_rad`：ROS ENU 坐标约定的 yaw，东为 0、逆时针为正；
- `pitch_deg`、`roll_deg`：HPR 原始俯仰和横滚；
- `solution_quality`：接收机 QF（1 单点、2 码差分、4 RTK 固定、5 RTK 浮点等）；
- `data_valid`：报文通过当前校验策略，且角度字段、QF 和新鲜度均有效时为 `true`。

注意：QF 是接收机的定位解状态，不是独立的航向精度指标。WTRTK-982DT 的双天线
定向不要求 NTRIP；在露天、两根天线固定且卫星可见条件良好时，单点状态也可以输出
航向。NTRIP 主要改善绝对定位和 RTK 解状态，并可间接改善复杂环境下的整体解算稳定性。

## 主要参数

默认参数在 `src/wtrtk_982dt_driver/config/gps_ntrip.yaml`。其中：

- `serial_port`、`baud_rate`：GNSS 串口及波特率；
- `gga_output_interval_sec`：GGA 定位输出周期，默认 `1.0` 秒（1 Hz）；
- `hpr_output_interval_sec`：GPHPR 定向输出周期，默认 `1.0` 秒（1 Hz）；
- `receiver_request_retry_sec`：没有收到对应报文时，重新发送输出命令的间隔；
- `attitude_timeout_sec`：HPR 超时判定，建议大于 HPR 输出周期的 2 倍；
- `heading_offset_deg`：天线基线航向到车辆前向的固定安装偏角；
- `validate_nmea_checksum`：是否拒绝校验和错误/缺失的 GGA/HPR；
- `ntrip_host`、`ntrip_port`、`ntrip_mountpoint`：caster 地址；
- `ntrip_version`：支持 `1.0`（兼容旧 caster）和 `2.0`；
- `gga_send_interval_sec`、`ntrip_reconnect_delay_sec`：上传及重连周期。

参数 `ntrip_username` / `ntrip_password` 也可在私有 YAML 中设置，其优先级高于环境
变量，但不要把含真实凭据的文件提交到版本库。题目中提供的旧凭据未复制进新代码。

## 频率与首次测试建议

协议的输出参数是“周期（秒）”，不是 Hz：`1`=1 Hz、`0.5`=2 Hz、`0.2`=5 Hz、
`0.1`=10 Hz。程序也允许 `0.05`=20 Hz，但应先确认设备固件和当前串口输出组合支持；
附件对 GPHPR 给出的明确命令示例是 `GPHPR 1`。程序默认定位和定向均为 1 Hz，便于先
验证接线、天线安装和消息内容。
稳定后可在 YAML 中设置：

```yaml
gga_output_interval_sec: 0.2   # 5 Hz
hpr_output_interval_sec: 0.1   # 10 Hz
attitude_timeout_sec: 0.5
```

这些命令每次打开串口时发送，不执行 `SAVECONFIG`，因此不会永久改写接收机配置。
若提高频率，请确认串口带宽足够，并以 `ros2 topic hz` 的实测结果为准。

首次测试建议在开阔室外进行：两根天线牢固安装、相对位置和基线长度固定，ANT1/ANT2
不要互换，避免人体或金属遮挡。先不配置 NTRIP，确认 `/gnss/fix` 有效且
`/gnss/attitude.data_valid=true`；随后再接入 NTRIP 验证 RTK 浮点/固定解。GPHPR 中
的 `roll_deg` 是接收机原始字段；在只靠一条双天线基线且没有完整姿态传感器约束时，
不要把它等同于高可信的三轴 IMU 姿态。

安装偏角可用已知朝向标定。例如车辆实际朝北时 `raw_heading_deg=90`，可先设置
`heading_offset_deg=-90`。偏角定义为：`车辆航向 = 原始航向 + heading_offset_deg`，
结果自动归一化到 `[0, 360)`。

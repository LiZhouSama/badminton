# 双设备串口解包与可视化（M1616M + TB100）

脚本：`run_dual_sensor_sage.py`

## 功能

- 实时读取 **M1616M 阵列压力传感器**（16x16）并显示热力图
- 实时读取 **TB100 IMU** 并解包出：
  - 加速度 `acc_g`（x, y, z）
  - 四元数 `quat_wxyz`
- 按主机接收时间做最近邻同步，给出同步时间差（ms）
- 可选保存同步 CSV

## 环境

WSL 侧使用你的 SAGE 环境：

```bash
conda activate SAGE
```

## 运行示例（Windows）

```bash
python run_dual_sensor_sage.py \
  --pressure-port COM10 \
  --imu-port COM4 \
  --pressure-range 1kg \
  --save-csv output
```

- `--pressure-port` 默认 Windows 下为 `COM10`
- `--imu-port` 默认 `COM4`
- 默认波特率：
  - 压力传感器：`115200`
  - TB100：`115200`

## 在 WSL2 运行（重点）

WSL2 不稳定支持直接打开 Windows 蓝牙串口或 USB 转串口。推荐路径是：

1. Windows 原生进程打开 `COM10` / `COM4`
2. Windows 侧转发为 TCP
3. WSL 侧 `run_dual_sensor_sage.py` 连接 TCP 并继续做解析、同步、保存和可视化

### 1. Windows 侧启动串口桥

在 Windows Terminal / PowerShell 中进入仓库目录：

```powershell
cd D:\a_WORK\Projects\PhD\tasks\badminton
python .\win_com_tcp_bridge.py --pressure-com COM10 --imu-com COM4
```

默认桥接：

- 压力传感器：`COM10 -> tcp://0.0.0.0:17010`
- IMU：`COM4 -> tcp://0.0.0.0:17004`

也可以从 WSL 里启动 Windows 侧 Python：

```bash
powershell.exe -NoProfile -Command "cd 'D:\a_WORK\Projects\PhD\tasks\badminton'; python .\win_com_tcp_bridge.py --pressure-com COM10 --imu-com COM4"
```

如果 Windows 防火墙弹窗，需要允许该 Python 进程在专用网络上监听。

### 2. WSL 侧运行采集/可视化

`wintcp://PORT` 会自动从 WSL 中解析 Windows host IP：

```bash
conda activate SAGE
python run_dual_sensor_sage.py \
  --pressure-port wintcp://17010 \
  --imu-port wintcp://17004 \
  --pressure-range 1kg \
  --save-csv output
```

在 WSL 中不传端口时，脚本默认也是：

- `--pressure-port wintcp://17010`
- `--imu-port wintcp://17004`

示例（仅压力调试）：

```bash
python -u run_dual_sensor_sage.py --disable-imu --headless --pressure-range 1kg
```

如果自动 Windows host IP 解析失败，手动查 Windows host IP：

```bash
ip route | awk '/default/ {print $3; exit}'
```

然后显式传 `tcp://IP:PORT`：

```bash
python run_dual_sensor_sage.py \
  --pressure-port tcp://172.xx.xx.1:17010 \
  --imu-port tcp://172.xx.xx.1:17004
```

也可以继续使用 `wintcp://PORT`，但显式覆盖 Windows host：

```bash
python run_dual_sensor_sage.py \
  --windows-host 172.xx.xx.1 \
  --pressure-port wintcp://17010 \
  --imu-port wintcp://17004
```

如果 WSL 侧报 `timed out`，Windows 桥已经显示 `listening`，优先检查：

1. Windows 防火墙是否允许当前 Python 监听 `17010` / `17004`
2. WSL 中 `ip route | awk '/default/ {print $3; exit}'` 得到的 IP 是否能连通
3. 是否有 VPN / 安全软件拦截 WSL2 虚拟网卡到 Windows host 的 TCP 连接

该模式下：串口独占打开和蓝牙/USB 驱动都留在 Windows，WSL 只接收原始字节流并做原有解析与可视化。

## 可选参数

- `--pressure-range 1kg`：启动时给压力模块发送量程设置（默认 1kg）
  - 可选：`1kg/3kg/5kg/10kg/20kg/30kg/50kg/skip`
  - 脚本会发送：`SETF=...` 然后 `SET=OK`
- `--pressure-cmd-suffix cr`：配置命令后缀（`none/cr/lf/crlf`），默认 `cr`（蓝牙串口下更稳）
- `--sync-max-dt-ms 200`：同步匹配最大时间差（毫秒）
- `--vmax 3000`：热力图颜色上限（单位 g）
- `--transpose --flipud --fliplr`：调整 16x16 显示方向

## 同步策略（时间轴对齐）

由于两个设备是独立蓝牙串口，脚本用 **主机接收时间戳** 对齐：

1. IMU 帧持续进入缓冲区
2. 每来一帧压力数据，选择时间上最近的 IMU 帧
3. 输出同步时间差 `SYNC lag (Pressure-IMU)`

这样可直接得到“压力帧 ↔ IMU帧”成对数据。

## 协议摘要

### M1616M

- 帧头：`AA AB AC`
- 长度：`516 bytes`
- 数据：`256点 * 2字节`（高字节在前）
- 校验：前515字节累加和低8位

### TB100

- 帧头：`AA 55`
- 长度字段：通常 `0x44`（内容68字节）
- 内容：信息字4字节 + 载荷64字节
- CRC：Modbus CRC16（低字节在前）
- 已解析字段：acc、quat（同时也解析了RPY/gyro/mag/temp）

---
如需我继续扩展：
- 保存为 `npz/h5` 高效二进制
- 增加实时曲线窗口（acc/quat）
- 接入 ROS2 发布话题
- 压力-IMU联合事件检测（击球瞬间等）

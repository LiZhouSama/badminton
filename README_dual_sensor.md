# 双设备串口解包与可视化（M1616M + TB100）

脚本：`run_dual_sensor.py`

## 功能

- 实时读取 **M1616M 阵列压力传感器**（16x16）并显示热力图
- 实时读取 **TB100 IMU** 并解包出：
  - 加速度 `acc_g`（x, y, z）
  - 四元数 `quat_wxyz`
- 按主机接收时间做最近邻同步，给出同步时间差（ms）
- 可选保存同步 CSV

## 环境

```bash
pip install pyserial numpy matplotlib
```

> 说明：你的环境里如果没有 `pip`，可先用你自己的 Python 环境（conda / venv）安装依赖后再运行。

## 运行示例（Windows）

```bash
python run_dual_sensor.py \
  --pressure-port COM8 \
  --imu-port COM4 \
  --pressure-range 1kg \
  --save-csv synced_output.csv
```

- `--pressure-port` 必填（请改成压力传感器蓝牙串口）
- `--imu-port` 默认 `COM4`
- 默认波特率：
  - 压力传感器：`115200`
  - TB100：`115200`

## 在 WSL 运行（重点）

- 现在脚本支持在 Linux/WSL 下直接写 `COMx`，会自动映射：
  - `COM8 -> /dev/ttyS7`
  - `COM4 -> /dev/ttyS3`
- 若 WSL 直连 `/dev/ttyS*` 报 `Input/output error`，`run_dual_sensor.py` 会自动回退到 `wincom://COMx`（通过 `win_serial_stdio.py` 走 Windows 串口）。

示例（仅压力调试）：

```bash
python -u run_dual_sensor.py --pressure-range 1kg --disable-imu
```

示例（双设备）：

```bash
python -u run_dual_sensor.py --pressure-port COM8 --imu-port COM4 --pressure-range 1kg
```

如果你希望显式使用桥接（而不是自动回退），可用性能脚本：

```bash
python run_dual_sensor_sage.py \
  --pressure-port wincom://COM8 \
  --imu-port wincom://COM4 \
  --win-python /mnt/c/Users/Lizhou/.conda/envs/SAGE/python.exe

# 仅调试压力（暂不接 IMU）
python run_dual_sensor_sage.py \
  --pressure-port wincom://COM8 \
  --disable-imu \
  --headless \
  --pressure-cmd-suffix cr
```

该模式下：采集逻辑仍在 WSL 跑，但串口由 Windows 子进程打开，更稳定。

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

# STM32F407 麦轮小车 — 通信协议参考手册

> **固件版本**：v3.0-ODOMTEST（26.08.29）
> **适用对象**：任何需要与小车下位机（STM32F407）交互的客户端 —— 客户端 App、微信小程序、ROS2 桥、调试工具
> **协议权威来源**：`Core/Src/main.c`（命令解析）、`Core/Src/telemetry.c`（遥测帧）、`Core/Src/ota.c`（OTA）、`tools/mecanum_bringup/mecanum_bringup/serial_bridge.py`（上位机参考实现）

---

## 目录

- [1. 物理层](#1-物理层)
- [2. 通用通信规则](#2-通用通信规则)
- [3. 命令总表](#3-命令总表)
- [4. 运动控制命令](#4-运动控制命令)
- [5. 遥测与里程计命令](#5-遥测与里程计命令)
- [6. 系统与诊断命令](#6-系统与诊断命令)
- [7. OTA 升级命令](#7-ota-升级命令)
- [8. 上行数据帧（下位机 → 上位机）](#8-上行数据帧下位机--上位机)
- [9. 坐标系与单位约定](#9-坐标系与单位约定)
- [10. CRC-8 算法](#10-crc-8-算法)
- [11. 解析示例](#11-解析示例)
- [12. 频率、时序与带宽](#12-频率时序与带宽)
- [13. 常见问题](#13-常见问题)

---

## 1. 物理层

| 项目 | 值 |
|------|-----|
| 串口外设 | USART2 |
| TX 引脚 | PD5 |
| RX 引脚 | PD6 |
| 波特率 | **460800** |
| 数据位 / 停止位 / 校验 | **8N1**（8 数据位、1 停止位、无校验） |
| 流控 | 无 |
| 发送方式 | TX：2048B 环形缓冲 + DMA1 Stream6 Ch4 异步发送（缓冲满时有限等待 20ms 后丢弃并计数） |
| 接收方式 | RXNE 中断 + 256B 环形缓冲 |

> ⚠️ **波特率是硬约束**。460800 下 STM32 分频误差 0.16%，客户端串口参数必须完全一致，否则乱码。

---

## 2. 通用通信规则

### 2.1 命令（上位机 → 下位机）

- 每条命令以 **回车 `\r` 或换行 `\n` 结尾**（建议发 `\r`）。
- **大小写不敏感**（`V,0.2,0,0` 与 `v,0.2,0,0` 等效）。
- **前导空格自动跳过**，参数之间的空格按解析函数容忍范围处理（`sscanf` 默认跳过空白）。
- 单条命令长度上限 127 字节（`cmd_buf[128]`）。
- 命令之间不需要额外握手；逐条发送即可，下位机主循环每 ~10ms 轮询一次接收缓冲。

### 2.2 响应（下位机 → 上位机）

- 成功：`OK ...`
- 失败：`ERR ...`
- 参数错误时，下位机通常直接返回 `ERR <用法说明>`。

### 2.3 与调试日志的区分

固件还有一套分级调试日志（`DEBUG` 命令控制等级），格式为：

```
[<tick_ms>] <TAG> <TYP> <消息>\r\n
例: [   12345] CHASSIS  INF chassis ready
```

- 日志等级：`NONE(0)` / `ERROR(1)` / `WARN(2)` / `INFO(3)`（默认）/ `DEBUG(4)` / `VERBOSE(5)`
- 客户端若按行解析，建议只认 `$T` / `$STAT` / `OK` / `ERR` / `VERSION:` 开头的数据行，忽略其余行。

---

## 3. 命令总表

| 分类 | 命令 | 说明 |
|------|------|------|
| 运动控制 | `V,<Vx>,<Vy>,<Wz>` | **核心命令**：麦轮速度矢量控制（m/s, m/s, rad/s） |
| 运动控制 | `W1~W4,<RPM>` / `WALL,<RPM>` | 单轮/全轮速度闭环 |
| 运动控制 | `W1~W4,DUTY,<duty>` / `WALL,DUTY,<duty>` | 单轮/全轮开环占空比（-1~1） |
| 运动控制 | `W1~W4,STOP` / `WALL,STOP` | 单轮/全轮停止 |
| 运动控制 | `FWD,<m/s>` / `BACK,<m/s>` / `LEFT,<deg/s>` / `RIGHT,<deg/s>` | 简单方向连续运动 |
| 运动控制 | `DUTY_FWD/BACK/LEFT/RIGHT,<0~1>` | 全车开环方向（无 PID 诊断用） |
| 运动控制 | `N/E/S/W,<speed>,<distance>` | 朝固定方向导航（自动停止） |
| 运动控制 | `DIR,<heading°>,<speed>,<distance>` | 任意方向导航（自动停止） |
| 运动控制 | `M1~M4,<RPM>` / `MALL,<RPM>` | 传统电机速度闭环（兼容） |
| 运动控制 | `M1~M4,DUTY,<duty>` / `MALL,DUTY,<duty>` | 传统电机开环 |
| 运动控制 | `M1~M4,STOP` / `MALL,STOP` | 传统电机停止 |
| 运动控制 | `PWM,<motor\|ALL>,<pulse>` | 直接 PWM 脉冲（-99~99） |
| 运动控制 | `STOP` | **紧急停止**（关闭全部电机输出） |
| 遥测 | `TELEM,ON` / `TELEM,OFF` | 开启/关闭遥测流 |
| 遥测 | `TELEM,<ms>` | 设置发送间隔并开启（0=最快, 20=50Hz, 100=10Hz） |
| 遥测 | `TELEM,NOW` | 立即按当前模式发一帧 |
| 遥测 | `TELEM,BIN` / `TELEM,ASCII` | 切换二进制帧 / ASCII 文本帧 |
| 遥测 | `ODORST` | 里程计归零（x=0, y=0，航向保持） |
| 遥测 | `ODOSET,<x>,<y>,<hdg>` | 设置里程计位姿（m, m, °） |
| 系统 | `STATUS` | 查询状态（返回 `$STAT` 帧） |
| 系统 | `VERSION` | 查询固件版本 |
| 系统 | `DEBUG` / `DEBUG,<0~5>` / `DEBUG,<名称>` | 调试日志控制 |
| 系统 | `AHRSMAG,ON` / `AHRSMAG,OFF` | 磁力计融合开关（默认 OFF） |
| 诊断 | `ENCVAL` | 读取 4 路编码器原始值 |
| 诊断 | `PIDTUNE,?` / `PIDTUNE,<m\|ALL>,<Kp>,<Ki>,<Kd>` | PID 参数查询/在线调整 |
| 诊断 | `PWMSCAN,<m\|ALL>,<start>,<end>,<step>,<hold_ms>` | 占空比扫描（阻塞式，回车中止） |
| 诊断 | `MAGCAL,<ms>` | 磁力计 2D 校准（阻塞式，需旋转 ≥360°） |
| 诊断 | `GYROCAL,<samples>` | 陀螺仪 Z 轴零偏校准（静止时执行） |
| OTA | `CMD_GET_VERSION` | 查询版本（供 OTA 决策） |
| OTA | `CMD_OTA_START` | 写升级标记 → 软复位进 Bootloader |

---

## 4. 运动控制命令

### 4.1 `V,<Vx>,<Vy>,<Wz>` — 麦轮速度矢量（核心）

机器人坐标系下直接给定速度矢量，连续运动，**需发 `STOP` 停止**。底层自动做麦轮逆运动学解算 + PID 速度闭环。

| 参数 | 含义 | 单位 | 范围（固件限幅） |
|------|------|:---:|:---:|
| `Vx` | 前后线速度（正=前进） | m/s | ±1.0 |
| `Vy` | 左右横移速度（正=左移） | m/s | ±1.0 |
| `Wz` | 旋转角速度（正=逆时针/左转） | rad/s | ±3.0 |

示例：

```
V,0.3,0.0,0.0   → 前进 0.3 m/s
V,-0.3,0.0,0.0  → 后退 0.3 m/s
V,0.0,0.3,0.0   → 左移 0.3 m/s
V,0.0,-0.3,0.0  → 右移 0.3 m/s
V,0.0,0.0,1.0   → 原地左转（逆时针）1.0 rad/s
V,0.2,0.3,0.5   → 斜移 + 旋转复合
```

> 下位机实际限幅与桥一致：`vx, vy ∈ [-1, 1] m/s`，`wz ∈ [-3, 3] rad/s`。超出会被夹紧。
> **客户端建议**：连续运动指令请周期重发（如每 100~200ms 一次），并在用户松手/停止操作时立即发 `STOP`。

### 4.2 单轮控制 `W1~W4` / `WALL`

| 子命令 | 说明 |
|--------|------|
| `W1,<RPM>` | 轮 1 速度闭环（M1=左前, M2=右前, M3=左后, M4=右后） |
| `W1,DUTY,<duty>` | 轮 1 开环占空比，duty ∈ [-1, 1] |
| `W1,STOP` | 停止轮 1 |
| `WALL,<RPM>` / `WALL,DUTY,<duty>` / `WALL,STOP` | 四轮统一 |

### 4.3 简单方向控制（连续运动）

| 命令 | 参数 | 说明 |
|------|------|------|
| `FWD,<speed>` | m/s | 前进 |
| `BACK,<speed>` | m/s | 后退 |
| `LEFT,<angular>` | deg/s | 原地左转 |
| `RIGHT,<angular>` | deg/s | 原地右转 |

### 4.4 导航模式（自动停止）

| 命令 | 参数 | 说明 |
|------|------|------|
| `N,<speed>,<distance>` | m/s, m | 朝北（0°）直行 `distance` 米后自动停 |
| `E,<speed>,<distance>` | m/s, m | 朝东（90°） |
| `S,<speed>,<distance>` | m/s, m | 朝南（180°） |
| `W,<speed>,<distance>` | m/s, m | 朝西（270°） |
| `DIR,<heading>,<speed>,<distance>` | °, m/s, m | 任意航向 |

> 参数保护：`speed ≤ 0` 时强制 0.5；`distance ≤ 0` 时强制 0.5。底盘忙（正在执行任务）返回 `ERR Busy`。

### 4.5 传统电机命令 `M1~M4` / `MALL`（兼容）

与 `W` 系列语法完全一致：`M1,<RPM>` / `M1,DUTY,<duty>` / `M1,STOP` / `MALL,...`。区别是 `M` 直接控制电机、不经过麦轮运动学，仅用于调试兼容。

### 4.6 `PWM,<motor|ALL>,<pulse>` — 直接脉冲

`pulse` ∈ [-99, 99]，正=前进、负=后退，直接写定时器比较值（0~99/99 即 0~100% 占空比）。

### 4.7 `STOP` — 紧急停止

停止底盘状态机并关闭全部电机输出。**所有客户端必须提供明显的急停入口**。

---

## 5. 遥测与里程计命令

### 5.1 `TELEM` — 遥测流控制

| 命令 | 行为 |
|------|------|
| `TELEM,ON` | 开启遥测流（保持当前帧模式，默认 ASCII `$T`，50Hz） |
| `TELEM,OFF` | 关闭遥测流 |
| `TELEM,<ms>` | 设置发送间隔并开启（0=每周期都发, 20=50Hz, 100=10Hz） |
| `TELEM,NOW` | 立即按当前模式发送一帧（不改变开关状态） |
| `TELEM,BIN` | 切换**二进制帧**（0xAA 0x55 + CRC8，112B，含原始 IMU/编码器 + EKF 诊断）并开启 |
| `TELEM,ASCII` | 切换 **ASCII 文本帧**（`$T`，31 字段）并开启 |

> **推荐客户端**：正式接入用 `TELEM,BIN`（定长 + CRC 校验，可靠且带宽省）；调试阶段用 ASCII。
> 固件上电默认：**遥测关闭**，需主动发 `TELEM,ON` 或 `TELEM,BIN` 开启。

### 5.2 `ODORST` / `ODOSET`

| 命令 | 行为 |
|------|------|
| `ODORST` | 里程计归零（x=0, y=0，航向保持） |
| `ODOSET,<x>,<y>,<hdg>` | 设置里程计位姿（x/y 单位 m，hdg 单位 °） |

---

## 6. 系统与诊断命令

| 命令 | 响应/行为 |
|------|-----------|
| `STATUS` | 立即返回一帧 `$STAT,...`（见 8.3） |
| `VERSION` | `Firmware: v3.0-ODOMTEST (26.08.29)` |
| `DEBUG` | 打印系统信息（时钟、调试等级、运行时间） |
| `DEBUG,4` / `DEBUG,VERBOSE` | 设置日志等级（数字 0-5 或名称） |
| `AHRSMAG,ON` | 开启磁力计融合（立即用磁航向重置 yaw）；默认 OFF（纯陀螺积分） |
| `AHRSMAG,OFF` / `AHRSMAG` | 关闭 / 查询 |
| `ENCVAL` | 打印 4 路编码器 CNT、增量、RPM、占空比、模式 |
| `PIDTUNE,?` | 打印 4 电机当前 Kp/Ki/Kd |
| `PIDTUNE,ALL,<Kp>,<Ki>,<Kd>` | 统一调整 PID |
| `MAGCAL,<ms>` | 磁力计 2D 校准（硬铁+软铁）。**阻塞式**：需将车放水平面、缓慢旋转 ≥360°，结束后打印结果并应用。不保存掉电。 |
| `GYROCAL,<samples>` | 陀螺仪零偏重校准。**阻塞式**：需保持车完全静止。 |
| `PWMSCAN,...` | 占空比扫描找死区。**阻塞式**：期间按回车中止。 |

---

## 7. OTA 升级命令

### 7.1 App 侧（运行中的固件）

| 命令 | 响应 | 说明 |
|------|------|------|
| `CMD_GET_VERSION` | `VERSION:v3.0-ODOMTEST 26.08.29` | 上位机轮询判断是否需要升级 |
| `CMD_OTA_START` | `OK OTA reboot`（随后软复位） | 在 Flash 0x0800FC00 写升级标记（魔数 `0x31544F41` "AOT1"），软复位进入 Bootloader |

### 7.2 Bootloader 侧（升级模式，行命令 `\r\n`）

| 命令 | 响应 | 说明 |
|------|------|------|
| `UPDATE` | `OK BL ready` | 握手（进入升级模式） |
| `INFO` | `BL <ver> <date> app@0x08010000 size<=960K valid=YES/NO` | 查询 Bootloader/App 信息 |
| `ERASE` | `OK erased sectors 4-11` | 擦除 App 区（可选，烧写前） |
| `ADDR,<8位HEX>` | `OK @0x08010000` | 设置写入起始地址（App 固定 0x08010000） |
| `DATA,<HEX字符串>` | `OK <n>B @<addr>` | 写入数据块，**单行 ≤ 64B**（即 ≤128 个 HEX 字符） |
| `RUN` | `OK jumping` | 校验 App 有效 → 清除升级标记 → 跳转 App |

> Bootloader 波特率以实际版本为准（排障期间曾降到 115200，与 App 的 460800 相互独立）。参考工具：`tools/ota.py`。
> App 分区：0x08010000，最大 960KB。

---

## 8. 上行数据帧（下位机 → 上位机）

固件有三种主动上行数据：**ASCII 遥测帧 `$T`**、**二进制遥测帧**、**状态帧 `$STAT`**（每 500ms 自动发一帧）。另有 `OK/ERR` 命令响应与调试日志。

### 8.1 ASCII 遥测帧 `$T`（TELEM,ASCII，默认）

**共 31 个逗号分隔字段**（24 个基础字段 + 7 个 ODOM EKF 诊断字段），以 `\r\n` 结尾。所有浮点数值先乘倍数转整数传输，客户端按"除数"还原：

```
$T,<tick>,<enc1>,<enc2>,<enc3>,<enc4>,
   <accel_x>,<accel_y>,<accel_z>,
   <gyro_x>,<gyro_y>,<gyro_z>,
   <mag_x>,<mag_y>,<mag_z>,
   <x>,<y>,<hdg>,
   <vx>,<vy>,<wz>,
   <wheel1>,<wheel2>,<wheel3>,<wheel4>,
   <flags>,<q_xy>,<q_yaw>,<r_heading>,<cov_x>,<cov_y>,<cov_yaw>\r\n
```

| # | 字段 | 传输倍数 | 还原除数 | 实际单位 |
|:--:|------|:---:|:---:|------|
| 1 | `tick` | 1 | 1 | ms（系统运行时间） |
| 2~5 | `enc1~4` | 1 | 1 | 编码器本周期增量（原始计数） |
| 6~8 | `accel_x~z` | ×1000 | ÷1000 | g |
| 9~11 | `gyro_x~z` | ×100 | ÷100 | °/s |
| 12~14 | `mag_x~z` | ×100 | ÷100 | µT |
| 15~16 | `x, y` | ×1000 | ÷1000 | m（世界坐标） |
| 17 | `hdg` | ×10 | ÷10 | °（罗盘角, 0=北） |
| 18~19 | `vx, vy` | ×1000 | ÷1000 | m/s（机器人系） |
| 20 | `wz` | ×1000 | ÷1000 | rad/s |
| 21~24 | `wheel1~4` | ×10 | ÷10 | RPM（四轮实际转速） |
| 25 | `flags` | 1 | 1 | 诊断标志位（见 8.4） |
| 26~31 | `q_xy, q_yaw, r_heading, cov_x, cov_y, cov_yaw` | ×10000 | ÷10000 | EKF 诊断（见 8.4） |

> 注意：旧文档记载 24 字段，v3.0-ODOMTEST 起追加 7 个诊断字段（25~31）。**解析器应容忍字段 ≥ 25 即可，多出的字段按上述表格解析**。

### 8.2 二进制遥测帧（TELEM,BIN）

**定长 112 字节**，帧头 `0xAA 0x55`，**小端字节序**，尾部 CRC-8（多项式 0x07，初值 0）。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|:--:|:--:|------|------|------|
| 0 | 2 | header | uint8[2] | 固定 `0xAA 0x55` |
| 2 | 1 | type | uint8 | 固定 `0x02` |
| 3 | 2 | payload_len | uint16 | 固定 `109`（type..cov_yaw 字节数） |
| 5 | 4 | timestamp | uint32 | 系统运行时间 (ms) |
| 9 | 16 | enc_raw[4] | int32[4] | 4 路编码器原始 CNT |
| 25 | 6 | accel_raw[3] | int16[3] | 加速度计原始 LSB（量程 ±2g，16384 LSB/g） |
| 31 | 6 | gyro_raw[3] | int16[3] | 陀螺仪原始 LSB（量程 ±250dps，131 LSB/(°/s)） |
| 37 | 6 | mag_raw[3] | int16[3] | 磁力计原始 LSB（AK8963，0.15 µT/LSB） |
| 43 | 12 | pose[3] | float[3] | x(m), y(m), yaw(rad) —— yaw 为 ROS 约定：0=北, 逆时针正（= -罗盘角） |
| 55 | 12 | twist[3] | float[3] | vx(m/s), vy(m/s), wz(rad/s) —— 机器人系 |
| 67 | 16 | wheel_speed[4] | float[4] | 四轮转速 (RPM) |
| 83 | 1 | diag_flags | uint8 | 诊断标志位（见 8.4） |
| 84 | 3 | reserved | uint8[3] | 保留（0x00） |
| 87 | 24 | diag_floats[6] | float[6] | q_xy, q_yaw, r_heading, cov_x, cov_y, cov_yaw |
| 111 | 1 | crc8 | uint8 | CRC-8，**覆盖偏移 2~110 共 109 字节** |

**传感器换算**（与 `serial_bridge.py` 一致）：

```
加速度 (m/s²) = accel_raw / 16384.0 × 9.80665
角速度 (rad/s) = gyro_raw / 131.0 × π / 180
磁场 (µT)    = mag_raw × 0.15
```

### 8.3 状态帧 `$STAT`（STATUS 响应 + 每 500ms 自动上报）

```
$STAT,<state>,HDG=<deg>,X=<m>,Y=<m>,DIST=<m>,REM=<m>,Vx=<mps>,Vy=<mps>,Wz=<radps>,M1=<rpm>,M2=<rpm>,M3=<rpm>,M4=<rpm>\r\n
```

| 字段 | 含义 | 单位 |
|------|------|:---:|
| `state` | 底盘状态：`IDLE` / `TURNING` / `MOVING` / `MANUAL` / `DONE` | — |
| `HDG` | 融合航向角（罗盘角） | ° |
| `X, Y` | 里程计世界坐标 | m |
| `DIST` | 已行驶距离 | m |
| `REM` | 剩余距离（导航模式） | m |
| `Vx, Vy` | 机器人速度 | m/s |
| `Wz` | 旋转角速度 | rad/s |
| `M1~M4` | 各轮转速 | RPM |

### 8.4 ODOM EKF 诊断标志位（flags）

| bit | 掩码 | 含义 |
|:--:|:--:|------|
| bit0 | 0x01 | 机器人静止（ZUPT 可用） |
| bit1 | 0x02 | 航向测量通过质量监督并用于更新 |
| bit2 | 0x04 | 编码器转向与陀螺仪偏差过大（打滑） |
| bit3 | 0x08 | 水平加速度过大（急加减速/打滑） |
| bit4 | 0x10 | 零速修正（ZUPT）已激活 |

`q_xy / q_yaw`：自适应 EKF 动态过程噪声；`r_heading`：航向测量噪声；`cov_x / cov_y / cov_yaw`：EKF 位置/航向协方差。客户端可用 `cov_x/cov_y` 判断里程计置信度（数值越小越可信）。

---

## 9. 坐标系与单位约定

```
机器人坐标系（ROS 约定）:
    X = 前进方向（+）
    Y = 车体左侧（+）
    Wz = 绕 Z 轴，逆时针为正（左转 +）

世界坐标系:
    X = 北（开机前进方向为 0°）
    Y = 西（右手系）
    heading = 罗盘角：0°=北, 90°=东, 180°=南, 270°=西，顺时针增加

二进制帧中的 yaw（rad）: ROS 约定，逆时针为正，yaw = -heading（弧度）
```

**方向速记**：`V,0.3,0,0` 前进；`V,0,0.3,0` 左移；`V,0,0,1.0` 左转。客户端摇杆设计时：上=前(Vx+)、左=左移(Vy+)、顺时针转=负 Wz。

---

## 10. CRC-8 算法

二进制帧校验：CRC-8，多项式 `0x07`，初值 `0x00`，无反射，无最终异或（SMBus 风格）。

```python
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc
```

校验范围：偏移 2（type）到偏移 110（cov_yaw 末字节），共 109 字节；结果写入偏移 111。

---

## 11. 解析示例

### 11.1 ASCII `$T` 解析（Python）

```python
def parse_telemetry(line: str):
    p = line.strip().split(',')
    if p[0] != '$T' or len(p) < 25:
        return None
    return {
        'tick':       int(p[1]),
        'enc_delta':  [int(x) for x in p[2:6]],
        'accel_g':    [int(x) / 1000.0 for x in p[6:9]],
        'gyro_dps':   [int(x) / 100.0 for x in p[9:12]],
        'mag_uT':     [int(x) / 100.0 for x in p[12:15]],
        'x_m':        int(p[15]) / 1000.0,
        'y_m':        int(p[16]) / 1000.0,
        'heading':    int(p[17]) / 10.0,          # 度, 罗盘角
        'vx':         int(p[18]) / 1000.0,
        'vy':         int(p[19]) / 1000.0,
        'wz':         int(p[20]) / 1000.0,
        'wheel_rpm':  [int(x) / 10.0 for x in p[21:25]],
    }
```

### 11.2 二进制帧解析（Python）

```python
import struct

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def parse_binary(buf: bytes):
    if len(buf) < 112 or buf[0:2] != b'\xAA\x55' or buf[2] != 0x02:
        return None
    if buf[-1] != crc8(buf[2:111]):
        return None
    return {
        'tick':     struct.unpack_from('<I', buf, 5)[0],
        'enc_raw':  list(struct.unpack_from('<4i', buf, 9)),
        'accel':    list(struct.unpack_from('<3h', buf, 25)),
        'gyro':     list(struct.unpack_from('<3h', buf, 31)),
        'mag':      list(struct.unpack_from('<3h', buf, 37)),
        'pose':     list(struct.unpack_from('<3f', buf, 43)),   # x, y, yaw(rad)
        'twist':    list(struct.unpack_from('<3f', buf, 55)),   # vx, vy, wz
        'wheel':    list(struct.unpack_from('<4f', buf, 67)),   # RPM
        'flags':    buf[83],
        'diag':     list(struct.unpack_from('<6f', buf, 87)),
    }
```

---

## 12. 频率、时序与带宽

| 项目 | 值 |
|------|-----|
| 主循环周期 | ~10ms（命令轮询、状态机） |
| 控制频率 | ~100Hz（AHRS + 里程计 + 底盘 + PID，实测 dt） |
| 遥测默认频率 | 50Hz（`TELEM,<ms>` 可调） |
| `$STAT` 自动上报 | 每 500ms |
| ASCII 帧大小 | ~31 字段 ≈ 230B/帧，50Hz ≈ 11.5KB/s（约占 460800 带宽 20%） |
| 二进制帧大小 | 112B/帧，50Hz ≈ 5.6KB/s |
| 下位机接收缓冲 | 256B 环形（突发命令勿一次塞太多，建议命令间隔 ≥ 20ms） |

---

## 13. 常见问题

| 问题 | 处理 |
|------|------|
| 串口乱码 | 确认波特率 460800、8N1 与下位机一致 |
| 发命令无响应 | 命令必须以 `\r` 结尾；检查是否在 DEBUG,0（日志全关）下误以为无响应（`OK/ERR` 不受日志等级影响，应始终可见） |
| 无遥测数据 | 需先发 `TELEM,ON`（ASCII）或 `TELEM,BIN`（二进制）；固件默认遥测关闭 |
| 二进制流里偶尔出现可读字符 | 正常：`$STAT` 与日志混在二进制流中。客户端需按帧头 `0xAA 0x55` + 长度 + CRC 同步，忽略其他字节 |
| 车不动但收到 OK | 检查电机方向表、PWM 死区（PWMSCAN 扫描）、限幅是否把速度压到死区内 |
| 收到 `ERR Busy` | 底盘正在执行导航任务，先 `STOP` 再发新指令 |

---

*文档生成：2026-09-05 · 与固件 v3.0-ODOMTEST 对齐 · 配套文档：`Core/docs/整体项目介绍.md`*

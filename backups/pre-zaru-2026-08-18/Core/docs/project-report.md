# STM32F407 麦轮小车项目报告

| 项目 | 内容 |
| --- | --- |
| 项目名称 | V1 — Mecanum 四轮麦轮导航小车 |
| MCU | STM32F407ZGT6 @ 168MHz（8MHz HSE，PLLM=4/N=168/P=2） |
| 工具链 | STM32CubeMX 生成 + CMake + arm-none-eabi-gcc 13.3（hard-float，FPU 开启） |
| 报告日期 | 2026-08-10 |
| 状态 | 可编译、可烧录、可启动；电机速度闭环可用；导航链路已修复核心缺陷 |

---

## 1. 项目概述

本项目是一个**麦轮（Mecanum）四轮驱动**的自主导航小车固件，核心目标：

1. 通过 **IMU（MPU9250，陀螺仪+磁力计）+ 编码器** 数据融合实现航位推算与自主导航；
2. 麦轮运动学控制：`V,<Vx>,<Vy>,<Wz>` 任意方向平移/旋转，并支持单轮独立控制；
3. 通过串口与上位机交互：命令控制 + 50Hz 全量遥测帧。

固件结构（全部为裸机轮询 + 中断，无 RTOS）：

```
main.c          主循环：命令解析 → 控制周期 → 各模块更新 → 遥测/心跳/LED
├── uart_io     串口驱动：RX 中断环形缓冲；TX 2048B 环形缓冲 + DMA1 Stream6 异步发送
├── debug       分级日志系统（DEBUG,<0-5>）
├── mpu9250     九轴传感器驱动（I2C1 100kHz，含总线故障自恢复）
├── ahrs        航向互补滤波（陀螺仪 98% + 磁力计 2%）
├── odometry    里程计：麦轮正运动学 + 航位推算（编码器 + AHRS 航向）
├── pid         增量式 PID（速度环 / 航向环共用）
├── motor       电机驱动：编码器速度测量 + 速度闭环 + PWM/方向控制
├── chassis     底盘状态机：麦轮逆运动学 + 导航（转向→直行→停止）
└── telemetry   遥测帧打包（ASCII CSV，50Hz，默认关闭）
```

---

## 2. 硬件架构

### 2.1 外设分配

| 外设 | 用途 | 引脚 | 说明 |
| --- | --- | --- | --- |
| TIM1 | 编码器 M1（左前） | PA8 / PA9 | 编码器模式 TI12，16bit，x4 倍频 |
| TIM3 | 编码器 M2（右前） | PA6 / PA7 | 同上 |
| TIM4 | 编码器 M3（左后） | PB6 / PB7 | 同上 |
| TIM8 | 编码器 M4（右后） | PC6 / PC7 | 同上 |
| TIM2 | PWM 四路（10kHz） | PA0 / PA1 / PA2 / PA3 | 预分频 83，ARR=99（1% 分辨率） |
| I2C1 | MPU9250（100kHz） | PB8(SCL) / PB9(SDA) | 含 GPIO 位拆裂总线恢复 |
| USART2 | 上位机串口 | PD5(TX) / PD6(RX) | **460800 8N1**，TX 走 DMA |
| GPIOC13 | 板载 LED | PC13 | 1Hz 心跳闪烁 |

### 2.2 电机与驱动

- 电机型号：MC520P30_12V（13 PPR 编码器 @ 电机轴，30:1 减速比，输出轴 52 计数/转）
- 驱动芯片：TB6612 × 2（每片两路）
- 方向引脚（IN1/IN2）：

| 电机 | 角色 | IN1 | IN2 | 方向配置（2026-08-10 实测数据推导后定稿） |
| --- | --- | --- | --- | --- |
| M1 | 左前 FL | PB0 | PB1 | **REVERSE，invert=false**（编码器 A/B 反） |
| M2 | 右前 FR | PB2 | PB10 | FORWARD，invert=false（**硬件完全正常**） |
| M3 | 左后 RL | PB11 | PB12 | **REVERSE，invert=true**（编码器 A/B 反 + 电机引线反） |
| M4 | 右后 RR | PB13 | PB14 | FORWARD，**invert=true**（电机引线反） |

> ⚠️ **两个补偿维度相互独立，需分别实测**（2026-08-10 上电失控事故总结）：
> - `direction_sign=-1`：补偿"编码器 A/B 相接反"（计数方向与物理转向相反）——表现：正反馈失控，读数符号与物理转向相反；
> - `invert=true`：补偿"电机正负引线接反"（IN1=H/IN2=L 时转向与假设相反）——表现：正反馈失控，物理转向与命令相反；
> - 两者独立叠加（M2/M3 双反时读数恰好正常，但 M3 曾出现"读数 +30 实际反转"的隐蔽问题）；只补偿一半 → 正反馈失控，调 PID 参数无效。

---

## 3. 主循环与控制周期

```
while(1):
  1. 串口命令解析（非阻塞，\r/\n 分行）
  2. 实测控制周期 dt（HAL_GetTick 差值，替代硬编码 10ms，抗打印抖动）
  3. AHRS_Update(dt)     —— 陀螺仪快速通道 + 磁力计互补滤波
  4. Odom_Update(dt)     —— 编码器正运动学 + 航位推算
  5. Chassis_Update(dt)  —— 底盘状态机
  6. Motor_UpdateAll(dt) —— 编码器测速 + 速度 PID + PWM 输出
  7. Telemetry_Update()  —— 50Hz 遥测（默认关）
  8. 状态上报 500ms / 心跳 1s / LED 500ms
  HAL_Delay(10)
```

关键设计：**dt 实测**。串口打印、I2C 阻塞会造成周期抖动（10~30ms），PID 与航位推算全部使用实测 dt，保证控制不因打印而失稳。

---

## 4. 控制算法

### 4.1 增量式速度 PID（电机）

公式（`pid.c`）：

```
eₖ = setpoint − measure
Δu = Kp·(eₖ − eₖ₋₁) + Ki·eₖ·Δt + Kd·(eₖ − 2eₖ₋₁ + eₖ₋₂)/Δt
uₖ = clamp(uₖ₋₁ + Δu, −1, 1)
```

| 参数 | 值 | 语义（增量式） |
| --- | --- | --- |
| Kp | 0.001 | 阻尼角色（对 Δe 响应） |
| Ki | 0.015 | 比例角色（对 e 直接累积，300RPM 误差 → 22 周期到满量程） |
| Kd | 0.00001 | 误差加速度阻尼 |
| 输出 | ±1.0 | 即占空比，对应 10kHz PWM |

特性：无积分累加器 → 天然抗饱和；setpoint 突变 bumpless；稳态无静差（输出由累积值保持）。

### 4.2 麦轮运动学（`chassis.c` / `odometry.c`）

几何参数：轮径 65mm（R=0.0325m），半轴距 Lx=Ly=75mm，L = Lx+Ly = 0.15m。

逆运动学（指令 → 轮速）：

```
ω_FL = ( Vx − Vy − ωz·L ) / R
ω_FR = ( Vx + Vy + ωz·L ) / R
ω_RL = ( Vx + Vy − ωz·L ) / R
ω_RR = ( Vx − Vy + ωz·L ) / R
```

正运动学（轮速 → 机器人速度，里程计用）：

```
Vx  = (R/4)·( ω₁ + ω₂ + ω₃ + ω₄ )
Vy  = (R/4)·(−ω₁ + ω₂ + ω₃ − ω₄ )
ωz  = (R/4L)·(−ω₁ + ω₂ − ω₃ + ω₄ )
```

航位推算（世界系）：

```
x += (Vx·cosθ − Vy·sinθ)·dt
y += (Vx·sinθ + Vy·cosθ)·dt
θ  = AHRS 航向（互补滤波融合值）
```

限幅：线速度 ±1.0 m/s，角速度 ±3.0 rad/s，轮速 ±294 RPM。

### 4.3 航向融合（`ahrs.c`）

```
gyro_yaw = yaw + (gz − 零偏)·dt          # 短期准
mag_yaw  = atan2f(my, mx) + 磁偏角        # 长期准（绝对方向）
yaw = gyro_yaw + 0.02·(mag_yaw − gyro_yaw) # 互补滤波 α=0.98，含 0°/360° 环绕处理
```

I2C 故障容错：陀螺仪连续读取失败 → 保持上次航向；总线 BUSY 卡死 → SWRST → GPIO 位拆裂恢复 → 500ms 冷却期，全程不阻塞主循环。

### 4.4 底盘状态机

```
IDLE ──DIR/N/E/S/W──► TURNING ──航向误差<3°──► MOVING ──距离达标──► DONE
                        │                         │
                        └────────── STOP / V 指令 ─┴──► IDLE / MANUAL
MANUAL：Vx/Vy/ωz 连续控制，直到 STOP
```

- TURNING：原地旋转，航向 PID 输出 ωz，`SetMecanumDrive(0, 0, ωz)`
- MOVING：直行 + 航向微调（PID 输出修正 ωz），四轮平均距离判定停止
- 距离容差 2cm，航向容差 3°

---

## 5. 串口通信

### 5.1 物理层

- **460800 8N1**，无流控（APB1=42MHz 下分频误差 0.16%）
- **TX：2048B 环形缓冲 + DMA1 Stream6 Ch4 异步发送**（`uart_io.c`）
  - 打印不阻塞控制循环；DMA 完成中断自动续传回绕段
  - 缓冲满：每字节最多等 20ms，超时丢弃并计数（防 DMA 异常挂死主循环）
  - DMA 初始化失败：自动回退逐字节轮询发送
- RX：中断接收 + 256B 环形缓冲（带满标志与溢出处理）

带宽核算：遥测帧 ~200-230B × 50Hz ≈ 11KB/s，460800 下占用 ~22%，余量充足。

### 5.2 命令集（详见 `serial-commands.md`）

| 命令 | 功能 |
| --- | --- |
| `V,<Vx>,<Vy>,<Wz>` | 麦轮速度矢量（m/s, m/s, rad/s） |
| `W1..W4,<RPM>` / `WALL,<RPM>` | 单轮速度闭环 |
| `Wx,DUTY,<-1..1>` / `Wx,STOP` | 单轮开环 / 停止 |
| `N/E/S/W,<速度>,<距离>` | 朝北/东/南/西导航（自动停） |
| `DIR,<航向°>,<速度>,<距离>` | 任意方向导航 |
| `FWD/BACK/LEFT/RIGHT,<数值>` | 简单方向连续运动 |
| `M1..M4,<RPM>` / `MALL,<RPM>` | 电机速度闭环（legacy） |
| `Mx,DUTY,<v>` / `Mx,STOP` | 单电机开环 / 停止 |
| `DUTY_FWD/BACK/LEFT/RIGHT,<v>` | 全车开环诊断（无 PID） |
| `STOP` / `STATUS` | 急停 / 状态查询 |
| `TELEM,ON/OFF/<ms>/NOW` | 遥测开关/间隔/立即发一帧 |
| `ODORST` / `ODOSET,<x>,<y>,<hdg>` | 里程计复位 / 设定 |
| `MAGCAL,<ms>` | 磁力计 2D 校准（硬铁+软铁） |
| `PIDTUNE,<motor\|ALL>,<Kp>,<Ki>,<Kd>` | 在线调 PID |
| `PWMSCAN,...` / `PWM,...` / `ENCVAL` | 死区扫描 / 直接脉冲 / 编码器读数 |
| `DEBUG,<0-5>` | 日志等级 |

### 5.3 遥测帧（双模式：ASCII 调试 / 二进制 RK3588）

**模式切换：** `TELEM,ASCII`（默认, 人眼调试）与 `TELEM,BIN`（RK3588 解包）。

**ASCII 帧（$T, 默认）：** 23 个整数字段（浮点 ×倍数 转整数，避免 %f 开销）

```
$T,<tick>,<enc1..4 delta>,<accel xyz mg>,<gyro xyz 0.01dps>,
   <mag xyz 0.01uT>,<x_mm>,<y_mm>,<heading 0.1°>,
   <vx_mmps>,<vy_mmps>,<omega_mradps>,<wheel1..4 rpm*10>\r\n
```

**二进制帧（TELEM,BIN）：** 定长 **84 字节**，0xAA 0x55 帧头 + CRC8（poly 0x07），包含**原始 int16 传感器值**（accel/gyro/mag LSB）与原始编码器 CNT，满足 `要求.md` 规格（另增 mag_raw）。字段布局见 `serial-commands.md` 9.2 节，解析代码见 `tools/serial_bridge.py`。

---

## 6. 构建与烧录

### 6.1 构建

```bash
# 需要 arm-none-eabi-gcc 13.x（STM32CubeCLT / GNU Tools for STM32）
cmake -S . -B build-ninja -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-ninja -j8
```

资源占用（2026-08-10 Debug 构建）：

| 资源 | 占用 | 总量 | 占比 |
| --- | --- | --- | --- |
| FLASH | 88,728 B | 1 MB | 8.46% |
| RAM | 7,176 B | 128 KB | 5.47% |

> 注：仓库原 `build/` 目录残留了失败的 NMake 配置缓存，重新配置前先删除或改用新目录（如 `build-ninja`）。

### 6.2 烧录

ST-Link（SWD）经 STM32CubeProgrammer / CubeIDE / VS Code ST-Link 扩展烧录 `build-ninja/V1.elf`。

---

## 7. 问题与修复记录（2026-08-10 重点审计）

### 7.1 🔴 里程计三角函数近似失效 → 已修复 ✅

| 项 | 内容 |
| --- | --- |
| 位置 | `odometry.c`（旧 `FastCos/FastSin` 泰勒 3 项近似） |
| 现象 | 航向 >~40° 后 x/y 积分发散：180° 时 cos 算出 +0.124（真值 −1.0），270° 时 +10.4（真值 0），315° 时 +24.0（真值 0.707）→ 朝南/朝西导航里程计完全错误 |
| 根因 | 泰勒展开仅在 \|x\|≲0.7 rad 有效，输入却是 0~360° 全范围 |
| 修复 | 改用 libm `sinf/cosf`（CMake 已链接 `-lm`，F407 硬浮点，单次调用 ~1µs） |
| 验证 | 0/45/90/…/315° 全部精确；编译链接通过 |

### 7.2 🔴 AHRS atan 负数分支错误 → 已修复 ✅

| 项 | 内容 |
| --- | --- |
| 位置 | `ahrs.c`（旧 `FastAtan/FastAtan2`） |
| 现象 | 磁航向在部分象限偏置：atan(−1.1) 算出 −2.32 rad（真值 −0.83，**误差 85°**），atan(−3) 误差 37°，\|x\|=1 处两分支跳变 6° → 互补滤波绝对基准错，导航转向停在错误角度（容差仅 3°） |
| 根因 | \|x\|>1 且 x<0 分支返回 `−(π/2 − term)` 而 term<0，结果超出 atan 值域；`abs_x` 计算后未使用 |
| 修复 | 改用 libm `atan2f`（语义完全一致：返回 [−π, π]） |
| 验证 | 六组象限向量全部精确（45°/−45°/−135°/135°/108.4°/−89.4°） |

### 7.3 🔴 PID 复位时序导致起步冲击 → 已修复 ✅

| 项 | 内容 |
| --- | --- |
| 位置 | `motor.c` `Motor_SetSpeed` + `pid.c` `PID_Reset` |
| 现象 | 静止 → `M1,300` 或 `V,...` 首周期 duty 猛跳 64.5%（模拟：峰值 90.9%），打滑/齿轮冲击；2026-01-20 的"平滑起步"修复在此路径未生效 |
| 根因 | `PID_Reset` 用 `pid->setpoint` 初始化误差历史，但 `Motor_SetSpeed` 在复位**之后**才由 `Motor_Update` 更新 setpoint → 复位时误差历史=0，首次计算被当作"误差从 0 突变到 300" |
| 修复 | `Motor_SetSpeed` 先 `PID_SetSetpoint(&pid, rpm)` 再 `PID_Reset`（误差历史=新目标误差 → P/D 项首周期为 0） |
| 验证 | 模拟：首周期 duty **64.5% → 4.5%**，2s 内收敛 300 RPM，无振荡 |

### 7.4 🔴 遥测串口饱和 → 已修复 ✅

| 项 | 内容 |
| --- | --- |
| 位置 | `usart.c` / `uart_io.c` |
| 现象 | 115200 下遥测 11KB/s 撞满带宽（上限 11.5KB/s），逐字节阻塞发送把控制周期拖到 25-30ms |
| 修复 | ① 波特率 115200 → **460800**（分频误差 0.16%）；② TX 改 **2048B 环形缓冲 + DMA1 Stream6 Ch4 异步发送**（打印零阻塞、回绕续传、满缓冲 20ms 限时丢弃、DMA 失败回退轮询） |
| 验证 | 环形逻辑宿主机模拟：200 条消息 5184B 全部按序发出、0 丢失；编译链接通过 |

### 7.5 ⚪ MPU9250 WHO_AM_I —— 判定不改（2026-08-10 用户实测确认）

- 驱动检查 `whoami != 0x70`，`main.c` 提示文案写"expected 0x71"，两者不一致（真 MPU9250 为 0x71，MPU6500 为 0x70）。
- **用户实测硬件返回 0x70**（模块为 MPU6500 内核方案），检查逻辑按实测保留，无需修改。
- 若以后更换真 MPU9250 模块，需将检查改为接受 0x71（或同时接受 0x70/0x71）。

### 7.6 🟡 链接顺序：`-lm` 在对象之前（本次修复的连带问题）→ 已修复 ✅

- 现象：引入 `sinf/cosf/atan2f` 后链接报 `undefined reference`。
- 根因：工具链文件把 `-lm` 放在链接命令行最前（对象文件之前），GNU ld 单遍扫描不回头。
- 修复：`CMakeLists.txt` 的 `target_link_libraries` 增加 `m`（CMake 自动放在对象之后）。

### 7.7 📜 历史修复（2026-01-20，见 `pid-oscillation-fix.md`）

- `PID_Reset` 零历史清零 → 用当前测量值初始化误差历史（消除闭环振荡）；
- 硬编码 `dt=0.01` → 实测控制周期；
- `Motor_SetLeftRightSpeed` 同步 setpoint；日志文案修正。

### 7.8 🔴 M1/M4 方向失控（上电实测）→ 已修复 ✅

| 项 | 内容 |
| --- | --- |
| 位置 | `motor.c` `Motor_InitAll` 方向配置 |
| 现象 | `V,0.1,0,0` 后 M1/M4 全速失控（读数 −333/−331），M2/M3 读数正常（+28/+30）但整车倒行左移+旋转 |
| 根因 | 开环逐轮标定 + 两轮实机复测后定稿：**M1 仅编码器 A/B 反；M2/M3 硬件完全正常；M4 仅电机引线反**。M2/M3 初判\"双反\"均为误判——其原版配置本身带反转（M2 原版即 REVERSE+invert=true；M3 被同配置化），开环测试被配置反转干扰：物理反转+读数被翻正 → 误以为硬件双反。M2/M3 实际是正常硬件被双重反转配置整体反转（闭环稳定但物理反，读数假象正常） |
| 修复 | 两轮修正后定稿：M1=REVERSE+invert=false，M2=FORWARD+invert=false，M3=FORWARD+invert=false，M4=FORWARD+invert=true |
| 验证 | 实机复测：第 1 轮 M2 整体反转（硬件正常被配置双反转）；第 2 轮 M3 同 M2 情况 → 一并修正为完全正常配置；待最终复测 |

### 7.9 🟡 原始 IMU 数据采集 + 二进制遥测帧 → 已实现 ✅

| 项 | 内容 |
| --- | --- |
| 需求 | `要求.md`：原始编码器 + 原始 IMU + 融合位姿 + 轮速打包发送（0xAA 0x55 + CRC8）；此前遥测只有缩放后的 ASCII 值，原始 int16 未被采集 |
| 实现 | ① `mpu9250.c` 采集 accel/gyro/mag 原始 int16 LSB；② `odometry.c` 暴露原始字段；③ `telemetry.c` 新增二进制帧（84B, CRC-8 poly 0x07, 小端）；④ `uart_io.c` 新增二进制安全发送 `UART_IO_SendBytes`；⑤ `TELEM,BIN`/`TELEM,ASCII` 切换（默认 ASCII 便于调试） |
| 验证 | 固件编译通过；桥侧 Python 模拟帧打包/解析/CRC/错帧检测全部通过（与 C 端算法一致） |

### 7.10 🟡 里程计悬空冻结 + ROS2 串口桥 → 已实现 ✅

| 项 | 内容 |
| --- | --- |
| 冻结 | `odometry.c`：底盘 IDLE/DONE（无运动指令，含悬空单轮测试）时冻结 X/Y 积分，防止编码器空转污染位姿；航向仍由 AHRS 持续更新（失控兜底 + 调试便利） |
| 桥 | `tools/serial_bridge.py`：ROS2 串口桥重写版——双格式自动识别（二进制/ASCII）、原始 IMU → `/imu/imu407/data_raw`、407 融合航向 → `/imu/imu407/attitude`、磁力计 → `/imu/imu407/mag`、`/odom` 坐标系可配、`/stm32_tick` 时间对齐、串口自动重连、cmd_vel → V 命令 |
| 验证 | `py_compile` 通过；帧解析/CRC 与 C 端逐字节一致 |

### 7.12 🟠 运动学坐标系切换为 ROS 约定（X=前 Y=左）→ 已完成 ✅

| 项 | 内容 |
| --- | --- |
| 需求 | 用户要求固件按 ROS 规定：+x=前，+y=左，ωz 逆时针正；命令 `V,<Vx>,<Vy>,<Wz>` 中第一个是 Vx、第二个是 Vy、第三个是 Wz |
| 变更 | ① `chassis.c` 逆运动学：`w_FL=(-vx-vy-ωL)/R, w_FR=(vx-vy+ωL)/R, w_RL=(vx-vy-ωL)/R, w_RR=(-vx-vy+ωL)/R`（由原版 X=右/Y=前 旋转 90° 变换）；② `odometry.c` 正运动学同步变换（`vx=(R/4)(-w1+w2+w3-w4)`，`vy=-(R/4)(w1+w2+w3+w4)`）；③ 世界系 X=北/Y=西（右手系），投影 θ = -heading（罗盘角 → ROS yaw 逆时针正）；④ 桥改为命令/odom **直通**（删除 pose_invert_yaw/pose_swap_xy 参数），yaw_ros = -heading |
| 验证 | 编译通过；1000 组随机速度逆/正运动学互逆性 PASS；前进/左移/左转轮速模式 -++- / ---- / -+-+ 全部正确；待实机复测 |

### 7.11 🟡 里程计轮速符号 bug（direction_sign 轮子）→ 已修复 ✅
| 项 | 内容 |
| --- | --- |
| 位置 | `odometry.c` `Odom_Update` |
| 现象 | 悬空 V,0.2,0,0：STATUS 四轮全正读数但 Vx=-0.10/Vy=-0.10/Wz=0.65（应 Vx=+0.2/Wz≈0） |
| 根因 | `wheel_mps` 用原始 `enc_delta`（未乘 direction_sign）计算，而 STATUS 用已乘符号的 RPM——M1/M3（sign=-1）正常正转时里程计把它们算成负速度（实测数据反推 Vx=-0.100 与公式完全吻合） |
| 修复 | `wheel_mps` 改为由已含方向符号的 `wheel_rpm` 换算（rpm×2π/60×R），删除废弃的 `EncDeltaToRadPerSec` |
| 验证 | 编译通过；与 B 版实测数据逐项吻合 |

---

## 8. 上电与调试检查清单

```text
1. 上电串口（460800 8N1）应看到：
   "=== STM32F407 Motor Control System ===" → "System ready." → 命令菜单
2. MPU9250 若显示 I2C 错误/ID 错误 → 检查 PB8/PB9 接线（不阻塞启动）
3. DEBUG,4 → 打开调试日志观察心跳与 dt
4. 空载测试：W1,100  → 电机 1 平滑起步到 100 RPM（观察无冲击）
   PWMSCAN,1,0.05,0.4,0.05,500 → 测死区
5. 底盘测试（车轮悬空）：V,0.2,0.0,0 → 四轮同向；V,0,0,1.0 → 原地旋转
6. TELEM,ON → 上位机收帧，检查时间戳递增、编码器方向符号
7. 导航测试：先 MAGCAL,10000 校准，再 DIR,90,0.3,1.0
8. 方向符号异常 → 检查编码器 A/B 相或 M2 的 invert 配置
```

---

## 9. 已知风险与后续建议

| # | 风险/问题 | 影响 | 建议 |
| --- | --- | --- | --- |
| 1 | **航向环收敛慢**：增量式 PID 输出为累积值，作为"角速度指令"时误差为 0 输出不归零。模拟：90° 转向峰值角速度仅 0.07 rad/s，需 ~40s 进 3° 容差（带 PWM 死区时更糟：先不动、积够后猛甩） | 导航 TURNING 实用性差 | 改位置式 PID 输出 ωz（Kp=2 立即给出 3.14 rad/s），或对累积输出做"误差→0 时衰减"处理 |
| 2 | 遥测为 ASCII 帧，与 `要求.md` 规定的二进制帧（0xAA 0x55 + CRC8 + 原始 int16）不符 | 上位机按文档解析失败 | 二选一：改实现为二进制帧，或改需求文档 |
| 3 | I2C 负载：每 10ms 周期 AHRS（陀螺仪 6B + 磁力计 21B）+ Odom（再读全量）≈ 4-8ms | 控制周期被拖到 15-20ms | 磁力计降频到 10Hz；I2C 提到 400kHz；或 Odom 复用 AHRS 的 IMU 数据 |
| 4 | 栈 1KB（链接脚本 `_Min_Stack_Size=0x400`） | 嵌套中断 + printf 有溢出风险 | 提到 2-4KB |
| 5 | 磁偏角 `MAG_DECLINATION=0` | 非北京地区航向有固定偏置 | 按所在地设置（北京约 −7°） |
| 6 | 编码器 A/B 相方向、磁力计安装方向未经实车验证 | 方向/航向可能反 | 上电用 `ENCVAL` + `V,0.2,0,0` 实测确认 |
| 7 | 板载晶振必须为 8MHz（配置与 .ioc 一致） | 25MHz 晶振板启动即挂 | 换板时核对 |
| 8 | 数据融合为"互补滤波 + 航位推算"，非 EKF | 精度有限，长时间漂移 | 后续可升级 EKF（`要求.md` 目标） |

---

## 10. 文档索引

| 文档 | 内容 |
| --- | --- |
| `Core/docs/要求.md` | 原始需求（二进制遥测帧设计、麦轮控制要求） |
| `Core/docs/serial-commands.md` | 串口命令参考手册 v2.0（含二进制帧规格 9.2） |
| `Core/docs/pid-oscillation-fix.md` | 2026-01-20 PID 振荡分析与修复记录 |
| `Core/docs/project-report.md` | 本报告 |
| `tools/serial_bridge.py` | ROS2 串口桥（拷贝到 RK3588 运行, 依赖 rclpy/serial） |

---

*报告生成：2026-08-10。构建验证：`cmake --build build-ninja`（arm-none-eabi-gcc 13.3.1）。算法验证：宿主机 Python 数值模拟。*

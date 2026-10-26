# STM32F407 上电偶发卡死问题记录

> 日期：2026-08-25  
> 平台：STM32F407 麦轮底盘 + RK3588 ROS2 Jazzy  
> 固件结构：Bootloader @ 0x08000000，App @ 0x08010000  
> 现象：小车偶发上电后无反应，必须手动按复位键后 App 才能正常运行

---

## 1. 现象描述

| 项目 | 描述 |
|---|---|
| 发生概率 | 偶发，不是每次上电都出现 |
| 卡住表现 | 小车无动作、无正常遥测、系统无反应 |
| LED 状态 | 常亮 |
| 按复位键后 | 恢复正常 |
| 串口输出 | 当前尚未确认卡住时是否有输出 |
| 影响 | 每次开机需要人工干预，影响无人化/远程部署 |

---

## 2. LED 状态说明

根据当前固件逻辑：

| LED 状态 | 可能位置 |
|---|---|
| 200ms 快闪 | Bootloader 升级模式 |
| 1Hz 慢闪 | App 主循环正常运行 |
| 常亮 | 启动早期卡死，或 App 初始化过程中卡住 |
| 常灭 | 更早期卡死，需结合串口判断 |

本次卡住时观察到 **LED 常亮**，说明：

- 没有进入 Bootloader 升级模式
- 没有进入 App 主循环
- 大概率卡在启动早期

---

## 3. 可能原因

### 3.1 HSE 晶振 / PLL 上电启动失败（最大嫌疑）

```c
HAL_Init();
SystemClock_Config();   // HSE + PLL -> 168MHz
if (HAL_RCC_OscConfig(...) != HAL_OK) {
    Error_Handler();    // while(1) 卡死
}
```

上电瞬间 VDD 尚未稳定时，HSE 可能起振失败，进入 `Error_Handler`。  
手动复位时电源已经稳定，所以复位后正常。

特征：

- 卡死
- LED 状态取决于卡死时机
- 复位后恢复
- 若串口可用，App 的 `Error_Handler` 会输出：

```text
!!! FATAL ERROR at tick=... ms !!!
!!! System halted. Check hardware!
```

### 3.2 BOOT0 悬空或电平不稳定

若 BOOT0 未可靠接 GND：

- BOOT0 误为高时，芯片可能进入系统 Bootloader
- 表现为上电后无正常 App 运行
- 与手动复位无关时可能持续出现

### 3.3 Bootloader PA15 按键误触发

Bootloader 上电会检测 PA15：

```c
if (BL_ButtonPressed()) {
    BL_UpdateLoop();   // 停在升级模式
}
```

若上电瞬间 PA15 被误读为低电平，会停在升级模式。  
但本现象中 LED 常亮，不符合升级模式 200ms 快闪，因此概率较低。

### 3.4 电源上电斜率不够 / 复位电路不可靠

可能原因：

- 电机驱动与 MCU 共用电源，上电瞬间电压跌落
- 5V→3.3V LDO 上电较慢
- NRST 缺少 100nF 对地电容
- 缺少外部复位监控芯片

### 3.5 App 初始化阶段卡住

App 启动后会依次初始化：

```text
GPIO → TIM → USART → I2C → MPU9250 → AHRS → Chassis → Odom → Telemetry
```

若某个外设初始化异常，也可能停在主循环之前。  
其中 MPU9250 初始化/I2C 通信是重点怀疑对象。

---

## 4. 复现时如何定位

### 4.1 观察 LED

| LED 现象 | 结论 |
|---|---|
| 200ms 快闪 | Bootloader 升级模式，检查 PA15 / 升级标记 |
| 1Hz 慢闪 | App 正常，检查串口/上位机 |
| 常亮 | 卡在启动早期，重点怀疑 HSE/PLL 或 App 外设初始化 |
| 常灭 | 更早期卡死，重点查电源、复位电路、BOOT0 |

### 4.2 接串口观察

卡住时不要按复位，直接接串口：

```bash
sudo stty -F /dev/ttyS1 460800 cs8 -cstopb -parenb -echo
sudo cat /dev/ttyS1
```

重点看：

```text
BOOTLOADER ...            → 停在 Bootloader 升级模式
!!! FATAL ERROR ...       → Error_Handler，HSE/PLL 或 HAL 初始化失败
Firmware: v2.4 ...        → App 已经开始启动
```

### 4.3 硬件测量

| 测量点 | 正常值 |
|---|---|
| BOOT0 对 GND | 接近 0V，且有 10k 下拉 |
| NRST 对 GND 电容 | 100nF |
| 上电时 3.3V | 缓慢上升仍应稳定到 3.3V |
| 上电时 5V | 不能被电机拉低到 4.5V 以下太久 |
| HSE 晶振两端 | 起振后应有 8MHz 正弦/方波 |

---

## 5. 后续可选加固方案（本次未实施）

### 5.1 固件层

```c
HAL_Init();
HAL_Delay(50);    // 等电源稳定
SystemClock_Config();
```

HSE 启动重试：

```c
for (int i = 0; i < 3; i++) {
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK) break;
    HAL_Delay(20);
}
```

Error_Handler 中让 LED 快闪，方便识别卡死位置。

### 5.2 硬件层

- BOOT0 增加可靠 10k 下拉
- NRST 增加 100nF 对地电容
- 电机电源与 MCU 电源隔离/加电容
- 必要时增加外部复位芯片

---

## 6. 当前结论

- 现象为**上电偶发启动失败，手动复位后恢复**
- 最大嫌疑为 **HSE/PLL 上电启动失败** 或 **电源上电时序问题**
- 本次仅记录问题，不修改固件和硬件
- 下次复现时优先确认：

```text
1. LED 是否常亮
2. 串口是否输出 FATAL ERROR
3. 上电瞬间 3.3V/5V 是否稳定
```

---

## 7. 对系统影响

| 项目 | 影响 |
|---|---|
| 人工调试 | 需手动复位，影响较小 |
| 无人值守启动 | 无法接受，需要后续解决 |
| ROS2 上位机 | 卡住时串口无数据，上位机无法工作 |
| OTA 远程升级 | 若升级后首次上电卡死，会被误判为升级失败 |

# STM32F407 Bootloader 工程

本工程由 V1 应用工程（C:\dosk\STM32\V1）裁剪改造而来，是配合 RK3588（ROS）做串口 OTA 升级的 Bootloader。
上位机烧写工具：`tools/bl_flash.py`。

## Flash 分区（F407ZGT6 = 1MB，非均匀扇区）

| 区域 | 地址范围 | 扇区 | 大小 | 实际占用 |
|------|---------|------|------|---------|
| Bootloader | 0x08000000 ~ 0x0800FFFF | 0~3 | 64KB | ~9KB |
| 升级标记 | 0x0800FC00 | 3（末页） | 4B | 魔数 0x31544F41 ("AOT1" 小端) |
| V1 App | 0x08010000 ~ 0x080FFFFF | 4~11 | 960KB | ~92KB |

选 64KB 分区是为了与 F407 非均匀扇区对齐（扇区 4 是天然 App 起始边界），擦写永远整扇区对齐。

## 启动流程

```
上电
 ├─ PA15 按键按住 ~200ms ? ────────────────> 强制进升级模式 (任何状态都有效)
 ├─ 读 0x0800FC00 标记 == "AOT1" ？ ──是──> 进升级模式（打横幅 "BOOTLOADER ... ready"）
 │                                            └─ 升级成功 RUN 后清标记 -> 跳 App
 └─ 无标记 -> App 向量有效（SP 在 RAM、PC 在 App 区且 Thumb）？ ──是──> 静默跳转 0x08010000
                                              └─ 无效 -> 常驻升级模式
```

断电保护：传输中途断电，标记仍在，重启后继续留在升级模式，不会误跳半成品固件。

## 按键与状态灯（硬件）

- **PA15 按键**（内部上拉，按键接 GND，按下为低电平）：上电时按住 ~200ms 即强制进入升级模式，
  用于 App 不支持 `CMD_OTA_START` 或想手动升级时的兜底入口（优先级高于标记与 App 跳转）。
  若按键接 VCC，需将 gpio.c 中 PA15 的 Pull 改为 GPIO_PULLDOWN 并反转 bootloader.c 中的电平判断。
- **PC13 LED**：升级模式下以 200ms 周期闪烁心跳——板子在跑就一直闪；
  Flash 擦写期间 CPU 停顿灯会停住（整片擦约 1s），恢复后继续闪，用于判断升级过程中板子是否卡死。

## RK3588 OTA 完整时序

```
RK3588 (ROS)                              STM32F407
──────────────────────────────────────────────────────
30s 轮询 / 升级触发: CMD_GET_VERSION   ->  App 回 "VERSION:v2.1 26.08.15\r\n"
版本旧:              CMD_OTA_START     ->  App 写标记 + 软复位
                                     复位进 Bootloader，打 "BOOTLOADER ... ready" 横幅
RK 看到横幅 -> UPDATE 握手 -> ADDR,08010000 -> 分块 DATA,<hex> -> RUN
RUN 成功 -> 清标记 -> 跳转新固件
```

注意：App 收到 CMD_OTA_START 后先回 `OK OTA reboot` 再擦 Flash（擦除期间 CPU 停顿约 1s，期间勿发数据）。

## 升级协议（USART2, 460800 8N1, 行命令，\r 或 \n 结尾）

| 命令 | 说明 |
|------|------|
| `UPDATE` | 握手，回 `OK BL ready` |
| `INFO` | 查询 Bootloader 版本与 App 状态 |
| `ERASE` | 擦除 App 区（扇区 4~11） |
| `ADDR,<8位十六进制>` | 设置写入地址，如 `ADDR,08010000` |
| `DATA,<十六进制串>` | 写入数据（每行最多 64 字节），自动按扇区擦除 + 回读校验 |
| `RUN` | 校验并跳转 App，成功后清升级标记 |

## 工程结构

```
Core/Src/bootloader.c     跳转 / Flash 擦写 / 升级协议
Core/Src/main.c           启动流程（标记检查 -> 跳转或升级模式）
Core/Src/gpio.c           时钟使能 + PC13 LED（升级模式心跳）+ PA15 按键（强制升级）
Core/Src/usart.c          USART2 460800
Core/Src/stm32f4xx_it.c   仅 SysTick
STM32F407XX_FLASH.ld      FLASH 截为 64KB（0x08000000 起）
tools/bl_flash.py         上位机烧写脚本（pyserial）
```

已删除全部应用源文件（motor/mpu9250/ahrs/chassis/odometry/telemetry/pid/debug/uart_io），
HAL 中 TIM/I2C 模块已禁用。

## 编译与烧写

```bash
# 构建（输出 build/bootloader.bin，约 9KB）
#   本工程构建目录已清空（构建产物不入库），首次需自己配置：
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# 首次烧写（ST-Link）：build/bootloader.bin   -> 0x08000000
#                     <App 工程产物>          -> 0x08010000

# 之后日常升级（串口，无需调试器），App 产物来自主工程的 build-chassisfix：
python tools/bl_flash.py COM3 "C:\dosk\STM32\ODOM - 副本\firmware\V1_v3.4.8-WHEEL80.bin"
python tools/bl_flash.py COM3 --info      # 只查信息
```

## 与 V1 App 工程的配对约定

V1 侧已做的配套修改（勿随意还原，否则 App 无法经 Bootloader 启动）：

- `STM32F407XX_FLASH.ld`：FLASH ORIGIN = 0x08010000，LENGTH = 960K
- `system_stm32f4xx.c`：USER_VECT_TAB_ADDRESS + VECT_TAB_OFFSET = 0x10000（向量表重定位）
- `Core/Src/ota.c`：CMD_GET_VERSION / CMD_OTA_START 命令处理，标记地址/魔数须与本工程 bootloader.h 一致

若要修改标记地址或魔数，必须同步改两处：本工程 `Core/Inc/bootloader.h` 与 V1 工程 `Core/Inc/ota.h`。

## 历史

原应用工程的文档（要求 / 交接 / 进度记录 / project-report / serial-commands / pid-oscillation-fix）
已随应用代码删除，原件保留在 C:\dosk\STM32\V1\Core\docs\。

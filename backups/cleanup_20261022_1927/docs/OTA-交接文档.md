# OTA 升级方案 — 交接文档

> 日期：2026-08-16（第二次更新）
> 状态：⚠️ 排障中 — PRIMASK 坑已解决；长 DATA 行无响应问题有一个新线索（USB-TTL 460800 信号质量），V1.2.2 对照实验待测
> 项目：STM32F407 麦轮小车（V1 工作区 = App，`V1 - 副本` = Bootloader）

---

## 一、架构与 Flash 分区

```
Flash 1MB (F407ZGT6, 非均匀扇区)
┌──────────────────────────────────────────────────────────────┐
│ 扇区 0~3 (0x08000000~0x0800FFFF, 各16KB) → Bootloader (64KB)  │
│   当前 14844B (V1.2.2 对照版 @115200, 待验证)                 │
│   扇区 3 末页 0x0800FC00 → 升级标记 (魔数 0x31544F41 "AOT1")   │
├──────────────────────────────────────────────────────────────┤
│ 扇区 4   (0x08010000~0x0801FFFF, 64KB)  → App 起始            │
│ 扇区 5~11 (0x08020000~0x080FFFFF, 各128KB) → App 续           │
│   当前 App 91680B (V2.1.0)                                    │
└──────────────────────────────────────────────────────────────┘
```

| 工程 | 位置 | 链接地址 | 构建产物 |
| --- | --- | --- | --- |
| **App** | `C:\dosk\STM32\V1` | 0x08010000（`STM32F407XX_FLASH.ld`） | `build-ninja/V1_app_0x08010000.bin` |
| **Bootloader** | `C:\dosk\STM32\V1 - 副本` | 0x08000000（64KB） | `V1 - 副本/build_bl_new/bootloader.bin` |

---

## 二、升级协议（USART2, 行命令 `\r\n`）

| 命令 | 说明 |
| --- | --- |
| `CMD_GET_VERSION` | App 应答 `VERSION:<ver> <date>`（RK 轮询版本用） |
| `CMD_OTA_START` | App 写升级标记 → 软复位进 Bootloader |
| `UPDATE` | 握手 → `OK BL ready` |
| `INFO` | 查询 → `BL <ver> <date> app@0x08010000 size<=960K valid=YES/NO` |
| `ERASE` | 擦除 App 区（扇区 4~11）→ `OK erased sectors 4-11` |
| `ADDR,<8hex>` | 设置写入地址 → `OK @0x08010000` |
| `DATA,<hex>` | 写入数据（每行 ≤64B）→ `OK <n>B @<addr>` / `ERR write code=..` |
| `RUN` | 校验 App 有效 → 清标记 → 跳转（→ `OK jumping`） |

**Bootloader 启动流程**：按键(PA15)强制升级 → 有标记进升级 → App 有效静默跳转 → 无 App 常驻升级。

**波特率**：当前 V1.2.2 为 **115200**（对照实验）；App 正常通信 460800 不变（互不影响）。

---

## 三、构建方法

### Bootloader（V1 - 副本）

```bash
cmake -S "V1 - 副本" -B "V1 - 副本/build_bl_new" -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build "V1 - 副本/build_bl_new" -j8
arm-none-eabi-objcopy -O binary "V1 - 副本/build_bl_new/bootloader.elf" \
    "V1 - 副本/build_bl_new/bootloader.bin"
```

> ⚠️ 旧 `build_bl/` 缓存路径失效（工程拷贝导致 CMakeCache 路径不匹配），用 `build_bl_new`。
> ⚠️ 波特率在 `V1 - 副本/Core/Src/usart.c` 第 42 行（`huart2.Init.BaudRate`），改后重编。

### App（V1 工作区）

```bash
cmake --build build-ninja -j8
arm-none-eabi-objcopy -O binary build-ninja/V1.elf build-ninja/V1_app_0x08010000.bin
```

---

## 四、烧录流程

### 1. Bootloader（必须 ST-Link，OTA 不能烧自己）

```bash
STM32_Programmer_CLI -c port=SWD -w "V1 - 副本\build_bl_new\bootloader.bin" 0x08000000
```

### 2. App（OTA，免 ST-Link）

```bash
python ota.py COM3 --baud 115200 build-ninja\V1_app_0x08010000.bin   # Windows (波特率跟 bootloader 走!)
python3 ota.py /dev/ttyS1 build-ninja/V1_app_0x08010000.bin          # RK3588 (460800 原生UART)
```

工具 `tools/ota.py`（2026-08-16 重写）：
- `--list` 列出串口 / `--info` 查 bootloader / `--version` 查 app 版本
- `--erase` 烧前整片擦除 / `--no-run` 只烧不跳 / `--erase-only` 单独擦除
- `--diag` DATA 写入诊断（短行 4B + 长行 64B）— **排障核心工具**
- `--chunk 16|32|64` 分块大小（应急）
- `--baud` 波特率（默认 460800，跟 bootloader 版本走）
- 自动处理 Windows COM10+ 前缀、进度/速率统计、15s 擦除超时

---

## 五、排障记录（按时间）

### 坑 1：跳转后中断关闭 → app 卡死（✅ 已解决，V1.2.0+）

| 现象 | 根因 | 修复 |
| --- | --- | --- |
| OTA 跳转后灯常亮（旧版）/ 灭灯（诊断版） | `BL_JumpToApp` 里 `__disable_irq()` 后直接跳转，**PRIMASK 保持 1**——跳转≠真实复位（复位后 PRIMASK=0）。app 的 SysTick 中断被屏蔽 → `HAL_Delay` 死等 | ① bootloader V1.2.0+：跳转前 `__enable_irq()`（治本）；② app main 开头 `__enable_irq()`（防御） |
| 证据 | 旧版卡 `MPU9250_Init` 的 `HAL_Delay(100)`（灯已亮=常亮）；诊断版卡 `BootLedSignal` 的 `HAL_Delay(80)`（灯已灭） | ST-Link 直烧正常（真实复位）→ 只有 OTA 路径触发 |

### 坑 2：长 DATA 行无响应（⚠️ 未解决，最新线索指向 USB-TTL 高波特率信号质量）

| 现象 | 时间线 |
| --- | --- |
| 短 DATA(4B) 正常 `OK 00000004B`；长 DATA(64B=133字符) 无响应 `''` | 最初怀疑：F407 单字节缓冲 + HAL 轮询 → ORE 溢出 |
| V1.2.1 改寄存器级紧密轮询 RXNE + ORE 恢复 → **烧录后测试：长行依旧无响应** | ORE 假设被否定（或不是主因） |
| **新线索**：`--info` 输出 `Aug 16 202`（丢字符）+ `app@0x0801����=960K`（乱码）——**bootloader→PC 方向长字符串也出错** | 双向长串都错 → 位错概率随字节数累积 → **USB-TTL 在 460800 下信号质量/位错嫌疑最大** |
| **对照实验**：V1.2.2 降 **115200** 已编译，待烧录测试 | 115200 长行 OK → 实锤波特率问题，OTA 定版 115200；仍无响应 → 回到 bootloader 逻辑 |

**判断标准（V1.2.2 @115200）**：
```text
python ota.py COM7 --baud 115200 --diag
短 DATA OK + 长 DATA OK  → 实锤 USB-TTL 460800 信号问题 → OTA 用 115200
长 DATA 仍无响应          → 不是波特率 → 继续查 bootloader 接收/处理
```

**其他旁证**：RK3588 原生 UART（ttyS1）第一次 OTA 全程成功（100%）——原生 UART 信号质量好，与 USB-TTL 形成对照。

---

## 六、诊断工具链（已内置 ota.py）

| 命令 | 用途 |
| --- | --- |
| `python ota.py COM3 --info` | 确认 bootloader 版本 + app 有效性 + **观察输出有无乱码（信号质量体检）** |
| `python ota.py COM3 --erase-only` | 单独测 Flash 擦除（正常=Flash/电压 OK） |
| `python ota.py COM3 --diag` | 短/长 DATA 对比（长行无响应=接收/信号问题；短行也挂=写入问题） |
| `STM32_Programmer_CLI -c port=SWD -r32 0x08000000 16` | 确认 bootloader 向量表还在（排除误擦） |

---

## 七、App 侧配套（V1 工作区已改）

| 文件 | 改动 |
| --- | --- |
| `STM32F407XX_FLASH.ld` | FLASH ORIGIN=0x08010000, LENGTH=960K |
| `system_stm32f4xx.c` | `USER_VECT_TAB_ADDRESS` + `VECT_TAB_OFFSET=0x10000` |
| `Core/Src/ota.c` + `Core/Inc/ota.h` | `CMD_GET_VERSION` / `CMD_OTA_START`（写标记→软复位），挂接在 `ParseCommand` 开头（main.c:440） |
| `Core/Src/main.c` | ① main 开头 `__enable_irq()`（坑1防御）；② `BootLedSignal(n)` 启动阶段 LED 指示；③ `VERSION` 命令 + 开机横幅版本 |
| 版本 | 固件 `FIRMWARE_VERSION`/`FIRMWARE_DATE` 在 ota.h（v2.1）；main.c 同宏已对齐 |

---

## 八、待办 / 下一步

```text
□ 【当前】烧 V1.2.2 (115200) → python ota.py COM7 --baud 115200 --diag
   ├─ 长行 OK → OTA 定版 115200, 更新 ota.py 默认 --baud=115200, 文档定稿
   └─ 仍无响应 → 查 bootloader 接收/写入逻辑 (下一步: DEBUG 输出阶段标记 / 换 USB-TTL 对照)
□ OTA 全链路验证: --baud 115200 完整烧录 → 灯 6 闪 → 慢闪 → V2.1.0 横幅
□ （可选）RK3588 的 ota.py 同步更新（--diag/--chunk/--baud）
□ （可选）换 CP2102/FT232 USB-TTL 测 460800（若想保留高速升级）
□ （可选）Bootloader 接收改 DMA 空闲中断（彻底消除接收瓶颈）
□ 恢复主项目: 烧录 V2.1.0 → 三轴验证 → IMU 磁力计 → 建图 → nav2
```

---

## 九、关键文件索引

| 文件 | 说明 |
| --- | --- |
| `V1 - 副本/Core/Src/bootloader.c` | Bootloader 主体（V1.2.2：紧密轮询 RXNE + `__enable_irq` 跳转 + 115200） |
| `V1 - 副本/Core/Inc/bootloader.h` | 分区/标记/协议定义，`BL_VERSION` 版本号 |
| `V1 - 副本/Core/Src/usart.c` | 波特率配置（第 42 行） |
| `V1 - 副本/build_bl_new/bootloader.bin` | 最新 bootloader 固件（14844B @115200） |
| `V1 - 副本/.workbuddy/memory/2026-08-15.md` | 方案设计笔记（第一版） |
| `tools/ota.py` | 上位机烧录工具（Windows/RK3588 通用，诊断齐全） |
| `Core/Src/ota.c` / `Core/Inc/ota.h` | App 侧 OTA 支持 |
| `build-ninja/V1_app_0x08010000.bin` | 最新 App 固件（91680B, V2.1.0） |
| `build/Debug/` | IDE(CubeIDE) 构建产物（bin 同样有效，已验证 main 断点） |

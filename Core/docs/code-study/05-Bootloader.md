# 05 Bootloader

## 1. 位置

```text
tools/Bootloader/Core/Src/main.c
tools/Bootloader/Core/Src/bootloader.c
```

## 2. 存储布局

```text
0x08000000 - 0x0800FFFF  Bootloader 64KB
0x08010000 - 0x080FFFFF  App 960KB
```

## 3. 启动流程

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_USART2_UART_Init();

if (BL_ButtonPressed()) { BL_UpdateLoop(); }
if (BL_UpdateFlagSet()) { BL_UpdateLoop(); }
if (BL_AppValid())      { BL_JumpToApp(); }
BL_UpdateLoop();   // 无有效 App
```

## 4. App 校验

```c
BL_AppValid():
    读取 0x08010000 的栈顶 SP
    读取 0x08010004 的复位向量 PC
    SP 必须在 SRAM 范围
    PC 必须在 App Flash 范围
    PC 必须是 Thumb 地址
```

## 5. 跳转

```c
HAL_UART_DeInit();
HAL_RCC_DeInit();
HAL_DeInit();
SysTick 关闭;
关闭中断;
SCB->VTOR = 0x08010000;
__set_MSP(sp);
__enable_irq();
app_reset();
```

## 6. 升级协议

```text
UPDATE     -> 握手
INFO       -> 查询版本/App是否有效
ERASE      -> 擦除 App 扇区
ADDR,<hex> -> 设置写入地址
DATA,<hex> -> 写数据
RUN        -> 校验并跳转
```

## 7. 升级标记

```c
BL_FLAG_ADDR  0x0800FC00
BL_FLAG_MAGIC 0x31544F41
```

## 8. 注意

- Bootloader 使用 PA15 按键进入升级模式。
- App 跳转不是真实复位，因此 App 入口需要主动 `__enable_irq()`。
- Flash 擦除时必须先解锁。

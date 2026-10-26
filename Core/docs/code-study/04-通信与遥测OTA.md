# 04 通信与遥测、OTA

## 1. 串口协议

命令格式：

```text
命令\r
```

例子：

```text
V,0.3,0,0
STOP
TELEM,ON
AHRSMAG,OFF
GYROCAL,1000
```

命令解析在：

```text
Core/Src/main.c -> ParseCommand()
```

---

## 2. 遥测帧

文件：

```text
Core/Src/telemetry.c
```

### 2.1 ASCII 帧 `$T`

```text
$T,<tick>,<enc1>,...,<accel>,<gyro>,<mag>,<x>,<y>,<heading>,<vx>,<vy>,<wz>,<wheel1..4>
```

用于人眼调试。

### 2.2 二进制帧

```text
0xAA 0x55 + type + len + timestamp + ...
```

优点：

```text
定长
CRC8
原始 int16
带宽更小
```

二进制帧布局：

```text
offset 0  : 0xAA 0x55
offset 2  : type
offset 3  : payload_len
offset 5  : timestamp
offset 9  : enc_raw[4] int32
offset 25 : accel_raw[3] int16
offset 31 : gyro_raw[3] int16
offset 37 : mag_raw[3] int16
offset 43 : pose[3] float
offset 55 : twist[3] float
offset 67 : wheel_speed[4] float
offset 83 : crc8
```

### 2.3 遥测开关

```text
TELEM,ON
TELEM,OFF
TELEM,ASCII
TELEM,BIN
TELEM,NOW
TELEM,100
```

---

## 3. OTA 升级

文件：

```text
Core/Src/ota.c
```

### 3.1 命令

```text
CMD_GET_VERSION
CMD_OTA_START
```

### 3.2 流程

```text
RK 发 CMD_OTA_START
App 回复 OK OTA reboot
App 擦写 0x0800FC00 写升级标记
App 软复位
Bootloader 检测标记
进入升级模式
等待 RK 通过 UPDATE/ADDR/DATA/RUN 写 App
RUN 后清标记并跳转 App
```

### 3.3 标记

```c
OTA_FLAG_ADDR  0x0800FC00
OTA_FLAG_MAGIC 0x31544F41
```

---

## 4. 常见坑

- ASCII 帧中磁力计经过 serial_bridge 解析后可能被放大 10000 倍。
- 实际使用建议用 `TELEM,BIN`。
- OTA 擦 Flash 期间 CPU 会短暂停顿，需要先回复再擦除。

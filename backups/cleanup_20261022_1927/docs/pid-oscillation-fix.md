# PID 闭环振荡问题分析与修复

## 现象

上电后发送速度闭环指令（如 `M1,300`、`FWD,0.5`），电机来回摆动，无法正常启动。

## 根因

**`PID_Reset()` 将增量式 PID 的误差历史 (`prev_error` / `prev2_error`) 清零，导致复位后第一次 `PID_Compute` 时 D 项（微分项）和 P 项产生剧烈冲击。**

### 增量式 PID 公式

```
eₖ  = setpoint − measure
Δu  = Kp·(eₖ − eₖ₋₁)  +  Ki·eₖ·Δt  +  Kd·(eₖ − 2eₖ₋₁ + eₖ₋₂)/Δt
uₖ  = clamp(uₖ₋₁ + Δu, −1, 1)
```

其中 `eₖ₋₁` 和 `eₖ₋₂` 分别存储在 `prev_error` 和 `prev2_error`。

### 修复前的 `PID_Reset`

```c
pid->prev_error  = 0.0f;   // ❌ 清零
pid->prev2_error = 0.0f;   // ❌ 清零
```

### 振荡推演

电机静止，目标 300 RPM，实际参数 Kp=0.001, Ki=0.015, Kd=0.00001, dt=0.01：

**第 1 周期：**

| 项 | 计算 | 结果 |
|----|------|------|
| error | 300 − 0 | 300 |
| P 项 | 0.001 × (300 − **0**) | **+0.30** ← 本应为 0 |
| D 项 | 0.00001 × (300 − **0** + **0**) / 0.01 | **+0.30** ← 本应为 0 |
| I 项 | 0.015 × 300 × 0.01 | +0.045 |
| **Δu** | | **+0.645 → duty 跳到 64.5%** |

电机猛冲，加速到 ~200 RPM。

**第 2 周期：**

| 项 | 计算 | 结果 |
|----|------|------|
| error | 300 − 200 | 100 |
| P 项 | 0.001 × (100 − 300) | −0.20 |
| D 项 | 0.00001 × (100 − 600 + 0) / 0.01 | **−0.50** ← 反向冲击 |
| I 项 | 0.015 × 100 × 0.01 | +0.015 |
| **Δu** | | **−0.685 → duty 从 +64% 砸到 −4%** |

电机强行反转，下一周期又被拽回来……**正反交替，持续振荡。**

### 本质原因

增量式 PID 的 P 项和 D 项计算的是**误差的变化量**（一阶差分和二阶差分），完全依赖误差历史。把历史清零等价于告诉控制器："上一瞬间误差是 0，现在突然变成 300"，控制器便会输出巨量校正。正确的做法是告诉控制器："误差一直是 300，没变过"，让 P/D 项输出为零，仅靠 I 项平滑推动。

## 修复

### `PID_Reset` 用当前测量值初始化误差历史

```c
void PID_Reset(PID_Controller *pid, float measurement)
{
    pid->prev_output = pid->output;
    float error = pid->setpoint - measurement;
    pid->prev_error  = error;   // 用当前误差, 不是 0
    pid->prev2_error = error;   // 用当前误差, 不是 0
}
```

修复后第一次 `PID_Compute`：

```
P 项 = Kp × (300 − 300)            = 0
D 项 = Kd × (300 − 600 + 300)/dt  = 0
I 项 = Ki × 300 × 0.01             = +0.045
─────────────────────────────────────────
Δu = +0.045  →  duty: 0 → 4.5% 平滑起步
```

### 附带修复

| 问题 | 修复 | 文件 |
|------|------|------|
| 硬编码 `dt=0.01f`，实际周期因串口打印可能达 45ms | 用 `HAL_GetTick()` 实测循环间隔 | `main.c` |
| `Motor_SetLeftRightSpeed` 不调用 `PID_SetSetpoint` | 每次设目标时同步 setpoint | `motor.c` |
| 日志声称 PID 参数与实际传入的不一致 | 修正日志字符串 | `motor.c` |

## 涉及文件

- `Inc/pid.h` — `PID_Reset` 签名增加 `measurement` 参数
- `Src/pid.c` — `PID_Reset` 实现修正
- `Src/motor.c` — `Motor_SetSpeed` / `Motor_Stop` 传入测量值；`Motor_SetLeftRightSpeed` 调用 `PID_SetSetpoint`；修正日志
- `Src/chassis.c` — 航向 PID 的 `PID_Reset` 传入 `-heading_error`
- `Src/main.c` — 实际 dt 测量替代硬编码

## 日期

2026-01-20

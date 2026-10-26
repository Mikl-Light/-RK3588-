/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* ============ main.c 是"总指挥所" ============
 * 它做两件事：
 *   1. main() 前半段：开机初始化（相当于"把全车的开关都打开一遍"）
 *   2. main() 后半段 while(1)：无限循环（相当于"之后每10ms把所有模块跑一圈"）
 *
 * 整个固件就是：左边开机做一次，右边之后不断重复。你学过的所有模块，
 * （串口/航向/电机PID/底盘/里程计）最终都被 main.c 这个"总指挥"每10ms串起来跑一遍。 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 引入自己写的各个模块的头文件（每个对应一个功能） */
#include "debug.h"       /* 调试日志 */
#include "pid.h"         /* PID 控制器 */
#include "motor.h"       /* 电机驱动 */
#include "uart_io.h"     /* 串口输入输出 */
#include "mpu9250.h"     /* 九轴传感器 */
#include "ahrs.h"        /* 航向估计 */
#include "chassis.h"     /* 底盘运动学 */
#include "odometry.h"    /* 里程计(EKF) */
#include "telemetry.h"   /* 遥测打包发送 */
#include "ota.h"         /* 在线升级 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/* 固件版本: 开机自动打印一次, 也可发送 VERSION 命令随时查询
 * 定义已移至 ota.h (RK OTA 侧 CMD_GET_VERSION 共用同一版本号)
 * v3.0-ODOMTEST (2026-08-29): ODOM 可靠里程计测试版 — 自适应EKF+监督层+ZUPT
 * V2.4.0 (2026-08-18): ZARU 零速零偏再估计 — 静止时自动重估陀螺零偏, 消除 HDG 匀速漂移
 * V2.0.0 (2026-08-12): 运动学 ROS 化 (Vx=前 Vy=左 Wz=逆时针正), IMU 旋转90°磁航向交换
 * V1.x   (2026-08-10): 电机方向标定定稿, 460800+DMA, 二进制帧, IDLE冻结, 符号修复 等 */

/* ZARU 零速零偏再估计: 四轮转速均低于该值 (RPM) 视为静止 */
/* 主循环靠这个判断"车是不是停了"，然后把结果告诉 AHRS 去做 ZARU */
#define ZARU_STATIONARY_RPM  1.0f


static char cmd_buf[128];       /**< 串口命令缓冲区：收到的命令字符先攒在这里 */
static uint8_t cmd_idx = 0;     /**< 缓冲区写入位置：已经攒到第几个字符了 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ParseCommand(const char *cmd);
static void ReportStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/* 【中文名】主函数：程序的入口，从这里开始执行 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 防御：bootloader 跳转过来时中断可能是关的（不是真复位），主动打开。
   * 否则系统滴答时钟会被屏蔽，HAL_Delay 会死等（这是 OTA 踩过的坑） */
  __enable_irq();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* 初始化 HAL 库（芯片底层库），它负责 sysTick 滴答时钟、Flash 等基础 */
  HAL_Init();

  /* 配置系统时钟到 168MHz */
  SystemClock_Config();

  /* ================================================================ */
  /* 第一步：初始化所有硬件外设（静默，此时 UART 尚不可用）            */
  /* ================================================================ */
  /* MX_xxx_Init 是 CubeMX 自动生成的函数，每个负责初始化一个外设：
   * GPIO=引脚, TIM=定时器(编码器/PWM), USART=串口, I2C=总线 */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* ================================================================ */
  /* 第二步：初始化串口 IO（printf 重定向）+ 调试日志系统              */
  /* ================================================================ */
  /* USER CODE BEGIN 2 */
  UART_IO_Init();                 /* 串口准备好（之后才能打印） */
  Debug_Init(DEBUG_LEVEL_INFO);   /* 调试日志系统开起来 */

  /* ================================================================ */
  /* 第三步：打印启动信息和初始化各个模块（此时 UART 已就绪）          */
  /* ================================================================ */
  UART_IO_Printf("\r\n=== STM32F407 Motor Control System ===\r\n");
  UART_IO_Printf("Firmware: " FIRMWARE_VERSION " (" FIRMWARE_DATE ", ROS kinematics: Vx=fwd Vy=left)\r\n");
  UART_IO_Printf("System Clock: 168MHz\r\n");

  /* 打印各外设就绪的日志 */
  DBG_INFO(TAG_MAIN,  "All peripherals initialized.");
  DBG_INFO(TAG_GPIO,  "GPIO initialized (PA/PB/PC/PD/PH)");
  DBG_INFO(TAG_TIM,   "TIM1 initialized (Encoder M1)");
  DBG_INFO(TAG_TIM,   "TIM2 initialized (PWM all motors)");
  DBG_INFO(TAG_TIM,   "TIM3 initialized (Encoder M2)");
  DBG_INFO(TAG_TIM,   "TIM4 initialized (Encoder M3)");
  DBG_INFO(TAG_TIM,   "TIM8 initialized (Encoder M4)");
  DBG_INFO(TAG_USART, "USART2 initialized (PD5=TX, PD6=RX, 460800 8N1, TX DMA)");
  DBG_INFO(TAG_I2C,   "I2C1 initialized (PB8=SCL, PB9=SDA, 100kHz)");

  /* 打印系统时钟详情 */
  DBG_INFO(TAG_SYSTEM, "HCLK=%lu Hz, PCLK1=%lu Hz, PCLK2=%lu Hz",
           (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)HAL_RCC_GetPCLK1Freq(),
           (unsigned long)HAL_RCC_GetPCLK2Freq());

  /* 初始化四个电机（配置编码器、PWM、PID） */
  DBG_INFO(TAG_MOTOR, "Initializing 4 motors...");
  Motor_InitAll();
  DBG_INFO(TAG_MOTOR, "Motors OK: 4 motors (TIM1/3/4/8 enc + TIM2 PWM)");
  UART_IO_Printf("Motors initialized: 4 motors (TIM1/3/4/8 encoder + TIM2 PWM)\r\n");

  /* 初始化 MPU9250 九轴传感器（加速度计+陀螺仪+磁力计） */
  UART_IO_Printf("Initializing MPU9250...\r\n");
  DBG_INFO(TAG_MPU, "Starting MPU9250 init (Gyro:250dps, Accel:2G, DLPF:42Hz, Mag:100Hz)...");
  int mpu_ret = MPU9250_Init(GYRO_FS_250, ACCEL_FS_2G, DLPF_42HZ, MAG_MODE_CONT_100HZ);
  if (mpu_ret == 0) {
    DBG_INFO(TAG_MPU, "MPU9250 init OK (Accel+Gyro+Mag)");
    UART_IO_Printf("MPU9250 OK! (Accel+Gyro+Mag)\r\n");

    /* 陀螺仪校准（保持传感器静止，采500个点求零偏） */
    DBG_INFO(TAG_MPU, "Calibrating gyroscope (500 samples, keep sensor still)...");
    UART_IO_Printf("Calibrating gyro (keep still)...\r\n");
    MPU9250_Calib calib;
    MPU9250_CalibrateGyro(500, &calib);
    UART_IO_Printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f\r\n",
                   calib.gyro_offset_x, calib.gyro_offset_y, calib.gyro_offset_z);
  } else if (mpu_ret == -1) {
    /* 各种失败原因分别提示 */
    DBG_ERROR(TAG_MPU, "I2C communication error! Check wiring (PB8=SCL, PB9=SDA)");
    UART_IO_Printf("MPU9250: I2C communication error!\r\n");
  } else if (mpu_ret == -2) {
    DBG_ERROR(TAG_MPU, "Wrong WHO_AM_I (expected 0x71)! Check MPU9250 chip");
    UART_IO_Printf("MPU9250: Wrong device ID (expected 0x71)!\r\n");
  } else if (mpu_ret == -3) {
    DBG_ERROR(TAG_MPU, "AK8963 magnetometer I2C error!");
    UART_IO_Printf("MPU9250: AK8963 magnetometer not found!\r\n");
  } else {
    DBG_ERROR(TAG_MPU, "AK8963 WIA mismatch (expected 0x48)!");
    UART_IO_Printf("MPU9250: AK8963 ID mismatch (expected 0x48)!\r\n");
  }

  /* 初始化航向参考系统 AHRS（读陀螺零偏、准备互补滤波） */
  DBG_INFO(TAG_AHRS, "Initializing AHRS...");
  AHRS_Init();
  DBG_INFO(TAG_AHRS, "AHRS initialized (complementary filter, alpha=0.98)");
  UART_IO_Printf("AHRS initialized.\r\n");

  /* 初始化底盘控制器（麦轮运动学 + 状态机，用默认配置） */
  DBG_INFO(TAG_CHASSIS, "Initializing chassis controller...");
  Chassis_Init(NULL);  /* NULL = 用默认麦轮配置 (65mm轮径, 150mm半轴距) */
  DBG_INFO(TAG_CHASSIS, "Chassis OK (Mecanum: Lx=75mm, Ly=75mm, max_spd=1.0m/s, max_ang=3.0rad/s)");
  UART_IO_Printf("Chassis initialized. (Mecanum: wheel=65mm, base=150x150mm)\r\n");

  /* 初始化里程计（编码器+IMU 融合，EKF） */
  DBG_INFO(TAG_MAIN, "Initializing odometry...");
  Odom_Init(NULL);
  DBG_INFO(TAG_MAIN, "Odometry OK (encoder+IMU dead reckoning)");

  /* 初始化遥测（默认关，等 TELEM,ON 命令才开） */
  DBG_INFO(TAG_MAIN, "Initializing telemetry...");
  Telemetry_Init(NULL);
  DBG_INFO(TAG_MAIN, "Telemetry OK (50Hz, disabled by default)");

  /* 打印"系统就绪" + 命令帮助菜单 */
  DBG_INFO(TAG_MAIN, "=== System ready ===");
  UART_IO_Printf("System ready.\r\n");
  UART_IO_Printf("=== Mecanum Velocity Control (continuous, use STOP to end) ===\r\n");
  UART_IO_Printf("  V,<Vx>,<Vy>,<Wz>  — Mecanum velocity vector (ROS convention)\r\n");
  UART_IO_Printf("  Vx=forward, Vy=left (m/s), Wz=CCW (rad/s)\r\n");
  UART_IO_Printf("  V,0.3,0.0,0.0   → forward 0.3m/s\r\n");
  UART_IO_Printf("  V,0.0,0.3,0.0   → strafe left 0.3m/s\r\n");
  UART_IO_Printf("  V,0.0,0.0,1.0   → rotate CCW 1.0rad/s\r\n");
  UART_IO_Printf("=== Individual Wheel Control ===\r\n");
  UART_IO_Printf("  W1,<RPM> ~ W4,<RPM>   WALL,<RPM>\r\n");
  UART_IO_Printf("  W1,DUTY,<-1..1>       W1,STOP\r\n");
  UART_IO_Printf("=== Navigation (auto-stop at distance) ===\r\n");
  UART_IO_Printf("  N,<m/s>,<m>   E,<m/s>,<m>   S,<m/s>,<m>   W,<m/s>,<m>\r\n");
  UART_IO_Printf("  DIR,<deg>,<m/s>,<m>\r\n");
  UART_IO_Printf("=== Motor Control (legacy) ===\r\n");
  UART_IO_Printf("  M1,<RPM> ~ M4,<RPM>   MALL,<RPM>\r\n");
  UART_IO_Printf("  M1,DUTY,<-1..1> (open loop)\r\n");
  UART_IO_Printf("  M1,STOP\r\n");
  UART_IO_Printf("=== Telemetry & Debug ===\r\n");
  UART_IO_Printf("  TELEM,ON     → enable 50Hz telemetry stream\r\n");
  UART_IO_Printf("  TELEM,OFF    → disable telemetry\r\n");
  UART_IO_Printf("  TELEM,<ms>   → set interval (0=fastest, 20=50Hz, 100=10Hz)\r\n");
  UART_IO_Printf("  TELEM,BIN    → binary frame (0xAA55+CRC8, 112B, raw IMU + ODOM diag)\r\n");
  UART_IO_Printf("  TELEM,ASCII  → text frame $T (default, for debugging)\r\n");
  UART_IO_Printf("  TELEM,NOW    → send one frame immediately\r\n");
  UART_IO_Printf("  ODORST       → reset odometry (x=0, y=0)\r\n");
  UART_IO_Printf("  ODOSET,<x>,<y>,<hdg> → set odometry pose\r\n");
  UART_IO_Printf("  MAGCAL,<ms> → magnetometer calibration (rotate 360°)\r\n");
  UART_IO_Printf("  ENCVAL       → read raw encoder counter values\r\n");
  UART_IO_Printf("  PIDTUNE,?    → query current PID parameters\r\n");
  UART_IO_Printf("  PWMSCAN,...  → PWM dead-zone scan\r\n");
  UART_IO_Printf("=== System ===\r\n");
  UART_IO_Printf("  STOP   STATUS   DEBUG   DEBUG,<0-5>\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 【中文名】主循环：从这里开始程序永远在转圈，每10ms把所有模块跑一遍 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ============================================================ */
    /* 1. 读取串口命令（从中断接收仓库取字节，攒成一行，遇回车就解析） */
    /* ============================================================ */
    while (UART_IO_Available() > 0) {   /* 接收仓库里还有字节吗？ */
        uint8_t ch;
        if (UART_IO_GetChar(&ch)) {     /* 取一个字节出来 */
            if (ch == '\r' || ch == '\n') {
                /* 遇到回车/换行 = 一行命令结束，拿去解析 */
                if (cmd_idx > 0) {
                    cmd_buf[cmd_idx] = '\0';   /* 补字符串结尾标志 */
                    ParseCommand(cmd_buf);      /* 解析并执行这条命令 */
                    cmd_idx = 0;                /* 清空，准备收下一条 */
                }
            } else if (cmd_idx < (int)sizeof(cmd_buf) - 1) {
                /* 普通字符：追加到命令缓冲区 */
                cmd_buf[cmd_idx++] = (char)ch;
            }
        }
    }

    /* ============================================================ */
    /* 2. 测量实际控制周期（不写死10ms，而是真测这一圈花了多久）     */
    /* ============================================================ */
    {
        static uint32_t last_loop_tick = 0;   /* 上一圈的时刻（static=跨循环保留） */
        uint32_t now_tick = HAL_GetTick();    /* 现在时刻 */
        float actual_dt = 0.01f;              /* 首周期默认按10ms算 */
        if (last_loop_tick > 0) {
            uint32_t elapsed_ms = now_tick - last_loop_tick;  /* 这一圈实际耗时 */
            if (elapsed_ms > 0 && elapsed_ms < 500) {
                actual_dt = (float)elapsed_ms * 0.001f;   /* 毫秒转秒 */
            }
        }
        last_loop_tick = now_tick;

        /* ② 检测车是否静止（四轮转速都≈0），把结果告诉 AHRS 做 ZARU */
        {
            int stationary = 1;   /* 先假设静止 */
            for (int i = 0; i < 4; i++) {
                float rpm = Motor_GetSpeed((uint8_t)i);
                if (rpm > ZARU_STATIONARY_RPM || rpm < -ZARU_STATIONARY_RPM) {
                    stationary = 0;   /* 有轮子在转 → 不是静止 */
                    break;
                }
            }
            AHRS_SetStationary(stationary != 0);   /* 通知 AHRS */
        }
        AHRS_Update(actual_dt);        /* 更新航向（陀螺积分+ZARU） */

        /* ③ 更新里程计（EKF 预测+更新） */
        Odom_Update(actual_dt);

        /* ④ 更新底盘状态机（麦轮运动学，决定四轮转速） */
        Chassis_Update(actual_dt);

        /* ⑤ 更新所有电机（PID 调速 + PWM 输出） */
        Motor_UpdateAll(actual_dt);
    }

    /* ============================================================ */
    /* 5. 遥测发送（TELEM,ON 时按间隔自动发帧）                      */
    /* ============================================================ */
    Telemetry_Update();

    /* ============================================================ */
    /* 6. 周期上报状态（每 500ms 调一次 ReportStatus）               */
    /* ============================================================ */
    {
        static uint32_t report_tick = 0;
        report_tick++;
        if (report_tick >= 50) {  /* 50 × 10ms = 500ms */
            report_tick = 0;
            ReportStatus();   /* 汇报当前状态（状态/航向/距离等） */
        }
    }

    /* ============================================================ */
    /* 7. 调试心跳（每 1s 打印一次存活信息）                        */
    /* ============================================================ */
    {
        static uint32_t heartbeat_tick = 0;
        static uint32_t last_heartbeat_ms = 0;
        heartbeat_tick++;
        if (heartbeat_tick >= 100) {  /* 100 × 10ms = 1s */
            heartbeat_tick = 0;
            uint32_t now = HAL_GetTick();
            uint32_t elapsed = now - last_heartbeat_ms;
            last_heartbeat_ms = now;
            DBG_DEBUG(TAG_MAIN, "Heartbeat alive | t=%lu ms | dt=%lu ms | heading=%.1f° | state=%s",
                      (unsigned long)now, (unsigned long)elapsed,
                      Chassis_GetCurrentHeading(), Chassis_GetStateName());
        }
    }

    /* ============================================================ */
    /* 8. 板载 LED 闪烁（PC13，每500ms翻转 → 1Hz 可见闪烁）         */
    /* ============================================================ */
    {
        static uint32_t led_tick = 0;
        led_tick++;
        if (led_tick >= 50) {  /* 50 × 10ms = 500ms → 1Hz blink */
            led_tick = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* 翻转LED电平 */
        }
    }

    HAL_Delay(10);   /* 延时10ms → 让主循环以约100Hz(10ms/圈)运行 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 解析串口命令并执行
 *
 * === 简单方向控制 (连续运动, 需 STOP 停止) ===
 *   FWD,<速度m/s>         → 前进
 *   BACK,<速度m/s>        → 后退
 *   LEFT,<角速度deg/s>    → 原地左转
 *   RIGHT,<角速度deg/s>   → 原地右转
 *
 * === 导航模式 (自动在目标距离停止) ===
 *   N,<速度>,<距离>       → 朝北
 *   E,<速度>,<距离>       → 朝东
 *   S,<速度>,<距离>       → 朝南
 *   W,<速度>,<距离>       → 朝西
 *   DIR,<航向度>,<速度>,<距离> → 任意方向
 *
 * === 单电机控制 ===
 *   M1,<RPM>              → 电机1 速度闭环
 *   M1,DUTY,<-1..1>       → 电机1 开环占空比
 *   M1,STOP               → 停止电机1
 *   MALL,<RPM>            → 全部电机速度闭环
 *   MALL,STOP             → 停止全部电机
 *
 * === 系统 ===
 *   STOP                   → 紧急停止
 *   STATUS                 → 查询状态
 */
/* 【中文名】解析命令（命令分发器）：收到一行文字命令，判断是什么命令，调用对应模块去执行。
 * 结构：一大串 if(strncmp(...)) 逐个比对命令开头，匹配到就执行然后 return。
 * strncmp = 比较字符串前几个字符是否相同（用来识别命令名） */
static void ParseCommand(const char *cmd)
{
    /* 跳过前导空格 */
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;   /* 空命令直接返回 */

    /* ================================================================ */
    /* OTA: RK3588 升级命令 (最高优先级, CMD_OTA_START 会软复位)        */
    /* ================================================================ */
    if (OTA_HandleCommand(cmd)) {   /* 交给 ota.c 处理，处理了就返回 */
        return;
    }

    /* ================================================================ */
    /* DEBUG: 调试指令                                                  */
    /* ================================================================ */
    if (Debug_ParseCommand(cmd)) {  /* 交给 debug 处理 */
        return;
    }

    /* ================================================================ */
    /* AHRSMAG: 磁力计融合开关 (默认 OFF 纯陀螺, 电机干扰时用 OFF)      */
    /*   AHRSMAG,ON / AHRSMAG,OFF / AHRSMAG (查询)                     */
    /* ================================================================ */
    if (strncmp(cmd, "AHRSMAG", 7) == 0) {
        const char *arg = cmd + 7;
        while (*arg == ',' || *arg == ' ') arg++;
        if (strncmp(arg, "ON", 2) == 0 || strncmp(arg, "on", 2) == 0) {
            AHRS_SetMagFusion(true);
            UART_IO_Printf("OK AHRS MAG ON\r\n");
        } else if (strncmp(arg, "OFF", 3) == 0 || strncmp(arg, "off", 3) == 0) {
            AHRS_SetMagFusion(false);
            UART_IO_Printf("OK AHRS MAG OFF\r\n");
        } else {
            UART_IO_Printf("AHRS MAG: %s\r\n",
                           AHRS_IsMagFusionEnabled() ? "ON" : "OFF (default)");
        }
        return;
    }

    /* ================================================================ */
    /* VERSION: 查询固件版本                                            */
    /* ================================================================ */
    if (strncmp(cmd, "VERSION", 7) == 0) {
        UART_IO_Printf("Firmware: " FIRMWARE_VERSION " (" FIRMWARE_DATE ")\r\n");
        return;
    }

    /* ================================================================ */
    /* V,<Vx>,<Vy>,<Wz>  — 麦轮速度矢量控制 (核心命令, ROS 约定)        */
    /*   例: V,0.2,0.0,0.5  → Vx=0.2m/s(前), Vy=0, ωz=0.5rad/s(左转)   */
    /*       V,0.3,0.0,0.0  → 前进 0.3m/s                              */
    /*       V,0.0,0.3,0.0  → 左移 0.3m/s                              */
    /*       V,0.0,0.0,1.0  → 原地左转 1.0rad/s (CCW)                  */
    /* ================================================================ */
    if (strncmp(cmd, "V,", 2) == 0) {
        float vx, vy, omega;
        if (sscanf(cmd, "%*[^,],%f,%f,%f", &vx, &vy, &omega) == 3) {
            Chassis_MoveMecanum(vx, vy, omega);
            UART_IO_Printf("OK Vx=%.3f Vy=%.3f Wz=%.3f\r\n", (double)vx, (double)vy, (double)omega);
        } else {
            UART_IO_Printf("ERR V,<Vx>,<Vy>,<Wz>  e.g. V,0.2,0.0,0.5\r\n");
        }
        return;
    }

    /* ================================================================ */
    /* TELEM: 遥测控制                                                   */
    /*   TELEM,ON / TELEM,OFF / TELEM,<interval_ms> / TELEM,NOW         */
    /* ================================================================ */
    if (strncmp(cmd, "TELEM", 5) == 0) {
        if (cmd[5] == ',' || cmd[5] == '\0' || cmd[5] == ' ') {
            const char *arg = cmd + 5;
            while (*arg == ',' || *arg == ' ') arg++;
            if (strncmp(arg, "ON", 2) == 0 || strncmp(arg, "on", 2) == 0) {
                Telemetry_SetEnabled(true);
                UART_IO_Printf("OK TELEM ON (50Hz)\r\n");
            } else if (strncmp(arg, "OFF", 3) == 0 || strncmp(arg, "off", 3) == 0) {
                Telemetry_SetEnabled(false);
                UART_IO_Printf("OK TELEM OFF\r\n");
            } else if (strncmp(arg, "NOW", 3) == 0 || strncmp(arg, "now", 3) == 0) {
                Telemetry_SendNow();
                UART_IO_Printf("OK TELEM sent one frame\r\n");
            } else if (strncmp(arg, "BIN", 3) == 0 || strncmp(arg, "bin", 3) == 0) {
                Telemetry_SetMode(TELEMETRY_MODE_BINARY);
                Telemetry_SetEnabled(true);
                UART_IO_Printf("OK TELEM BIN (0xAA55+CRC8, 112B, raw + ODOM diag)\r\n");
            } else if (strncmp(arg, "ASCII", 5) == 0 || strncmp(arg, "ascii", 5) == 0) {
                Telemetry_SetMode(TELEMETRY_MODE_ASCII);
                Telemetry_SetEnabled(true);
                UART_IO_Printf("OK TELEM ASCII ($T text)\r\n");
            } else {
                int interval;
                if (sscanf(arg, "%d", &interval) == 1) {
                    Telemetry_SetInterval((uint16_t)interval);
                    Telemetry_SetEnabled(true);
                    UART_IO_Printf("OK TELEM interval=%dms\r\n", interval);
                } else {
                    UART_IO_Printf("ERR TELEM,ON|OFF|<ms>|NOW\r\n");
                }
            }
        } else {
            UART_IO_Printf("ERR TELEM,ON|OFF|<ms>|NOW\r\n");
        }
        return;
    }

    /* ================================================================ */
    /* ODORST / ODOSET: 里程计控制                                       */
    /* ================================================================ */
    if (strncmp(cmd, "ODORST", 6) == 0) {
        Odom_Reset();
        UART_IO_Printf("OK Odometry reset (x=0, y=0)\r\n");
        return;
    }
    if (strncmp(cmd, "ODOSET,", 7) == 0) {
        float x, y, hdg;
        if (sscanf(cmd, "%*[^,],%f,%f,%f", &x, &y, &hdg) == 3) {
            Odom_SetPose(x, y, hdg);
            UART_IO_Printf("OK Odometry set: x=%.3f y=%.3f hdg=%.1f\r\n", (double)x, (double)y, (double)hdg);
        } else {
            UART_IO_Printf("ERR ODOSET,<x_m>,<y_m>,<heading_deg>\r\n");
        }
        return;
    }

    /* ================================================================ */
    /* MAGCAL: 磁力计 2D 平面校准 (硬铁 + 软铁)                          */
    /*   格式: MAGCAL,<duration_ms>                                      */
    /*   例:   MAGCAL,10000  → 采集 10 秒, 请水平旋转机器人 ≥360°        */
    /*   使用前: 将机器人放在水平面上, 远离大块金属                       */
    /*   使用中: 缓慢水平旋转机器人至少一整圈                             */
    /* ================================================================ */
    if (strncmp(cmd, "MAGCAL", 6) == 0) {
        uint32_t duration_ms = 10000;  /* 默认 10 秒 */
        if (cmd[6] == ',') {
            int dur = 0;
            if (sscanf(cmd + 7, "%d", &dur) == 1 && dur >= 3000 && dur <= 60000) {
                duration_ms = (uint32_t)dur;
            }
        }

        UART_IO_Printf("=== Magnetometer 2D Calibration ===\r\n");
        UART_IO_Printf("Duration: %lu ms\r\n", (unsigned long)duration_ms);
        UART_IO_Printf("Place robot on LEVEL surface.\r\n");
        UART_IO_Printf("SLOWLY rotate robot >= 360° in horizontal plane.\r\n");
        UART_IO_Printf("Calibrating... (LED will blink)\r\n");
        UART_IO_FlushTx();

        MPU9250_Calib calib;
        MPU9250_CalibrateMag(duration_ms, &calib);

        UART_IO_Printf("\r\n=== Calibration Results ===\r\n");
        UART_IO_Printf("Samples: many (min-max method)\r\n");
        UART_IO_Printf("Hard Iron:  X=%.1f  Y=%.1f uT\r\n",
                       (double)calib.mag_offset_x, (double)calib.mag_offset_y);
        UART_IO_Printf("Soft Iron:  X=%.3f  Y=%.3f\r\n",
                       (double)calib.mag_scale_x, (double)calib.mag_scale_y);
        UART_IO_Printf("OK MAG calibration applied!\r\n");
        return;
    }

    /* ================================================================ */
    /* GYROCAL: 手动重新校准陀螺仪 Z 轴零偏 (静止时执行)                */
    /*   格式: GYROCAL,<samples>                                        */
    /*   例:   GYROCAL,1000  -> 采集 1000 点，约 1 秒                  */
    /* ================================================================ */
    if (strncmp(cmd, "GYROCAL", 7) == 0) {
        uint16_t samples = 1000;  /* 默认 1000 点，约 1 秒 */
        if (cmd[7] == ',') {
            int n = 0;
            if (sscanf(cmd + 8, "%d", &n) == 1 && n > 0 && n <= 5000) {
                samples = (uint16_t)n;
            }
        }
        UART_IO_Printf("=== Gyro Z offset calibration ===\r\n");
        UART_IO_Printf("Keep robot COMPLETELY STILL.\r\n");
        UART_IO_Printf("Samples: %u (~%u ms)\r\n", (unsigned int)samples, (unsigned int)samples);
        UART_IO_FlushTx();

        AHRS_RecalibrateGyro(samples);

        UART_IO_Printf("OK GYROCAL done\r\n");
        return;
    }

    /* ================================================================ */
    /* W1~W4 / WALL: 单轮独立控制 (麦轮)                                 */
    /*   W1,<RPM> / W1,DUTY,<-1..1> / W1,STOP / WALL,<RPM>              */
    /* ================================================================ */
    if (cmd[0] == 'W' || cmd[0] == 'w') {
        int wheel_id = -1;   /* -1=ALL, 0=未匹配 */
        const char *sub = NULL;

        if      (strncmp(cmd, "W1,", 3) == 0 || strncmp(cmd, "w1,", 3) == 0) { wheel_id = 1; sub = cmd + 3; }
        else if (strncmp(cmd, "W2,", 3) == 0 || strncmp(cmd, "w2,", 3) == 0) { wheel_id = 2; sub = cmd + 3; }
        else if (strncmp(cmd, "W3,", 3) == 0 || strncmp(cmd, "w3,", 3) == 0) { wheel_id = 3; sub = cmd + 3; }
        else if (strncmp(cmd, "W4,", 3) == 0 || strncmp(cmd, "w4,", 3) == 0) { wheel_id = 4; sub = cmd + 3; }
        else if (strncmp(cmd, "WALL,", 5) == 0 || strncmp(cmd, "wall,", 5) == 0) { wheel_id = -1; sub = cmd + 5; }

        if (sub != NULL) {
            /* 已匹配到 W 命令, sub 指向子命令部分 */
            if (strncmp(sub, "STOP", 4) == 0) {
                if (wheel_id > 0) {
                    Chassis_StopWheel((uint8_t)wheel_id);
                    UART_IO_Printf("OK W%d STOP\r\n", wheel_id);
                } else {
                    Motor_StopAll();
                    UART_IO_Printf("OK WALL STOP\r\n");
                }
            } else if (strncmp(sub, "DUTY,", 5) == 0) {
                float duty;
                if (sscanf(sub, "%*[^,],%f", &duty) == 1) {
                    if (wheel_id > 0) {
                        Chassis_SetWheelDuty((uint8_t)wheel_id, duty);
                        UART_IO_Printf("OK W%d DUTY=%.2f\r\n", wheel_id, (double)duty);
                    } else {
                        for (uint8_t i = 1; i <= 4; i++)
                            Chassis_SetWheelDuty(i, duty);
                        UART_IO_Printf("OK WALL DUTY=%.2f\r\n", (double)duty);
                    }
                } else {
                    UART_IO_Printf("ERR Wx,DUTY,<-1..1>\r\n");
                }
            } else {
                float rpm;
                if (sscanf(sub, "%f", &rpm) == 1) {
                    if (wheel_id > 0) {
                        Chassis_SetWheelSpeed((uint8_t)wheel_id, rpm);
                        UART_IO_Printf("OK W%d RPM=%.0f\r\n", wheel_id, (double)rpm);
                    } else {
                        for (uint8_t i = 1; i <= 4; i++)
                            Chassis_SetWheelSpeed(i, rpm);
                        UART_IO_Printf("OK WALL RPM=%.0f\r\n", (double)rpm);
                    }
                } else {
                    UART_IO_Printf("ERR Wx,<RPM>\r\n");
                }
            }
            return;
        }
        /* W 开头但未匹配 W1~W4/WALL → 可能是导航 W, 继续往下匹配 */
    }

    /* ================================================================ */
    /* STOP: 紧急停止                                                    */
    /* ================================================================ */
    if (strncmp(cmd, "STOP", 4) == 0 && (cmd[4] == '\0' || cmd[4] == '\r' || cmd[4] == ' ')) {
        Chassis_Stop();
        UART_IO_Printf("OK STOP\r\n");
        return;
    }

    /* ================================================================ */
    /* STATUS: 查询状态                                                  */
    /* ================================================================ */
    if (strncmp(cmd, "STATUS", 6) == 0) {
        ReportStatus();
        return;
    }

    /* ================================================================ */
    /* ENCVAL: 读取编码器原始计数器值（诊断用）                          */
    /* ================================================================ */
    if (strncmp(cmd, "ENCVAL", 6) == 0) {
        UART_IO_Printf("=== Encoder Raw Counter Values ===\r\n");
        for (int i = 0; i < 4; i++) {
            uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(g_motor[i].htim_encoder);
            uint32_t pulse = __HAL_TIM_GET_COMPARE(g_motor[i].htim_pwm, g_motor[i].pwm_channel);
            UART_IO_Printf("M%d (TIM%d): CNT=%5u, delta=%6ld, speed=%.0f RPM, duty=%.3f, pulse=%lu/%lu, mode=%s\r\n",
                           i + 1,
                           (g_motor[i].htim_encoder == &htim1) ? 1 :
                           (g_motor[i].htim_encoder == &htim3) ? 3 :
                           (g_motor[i].htim_encoder == &htim4) ? 4 : 8,
                           cnt,
                           (long)g_motor[i].count_delta,
                           (double)g_motor[i].speed_rpm,
                           (double)g_motor[i].duty,
                           (unsigned long)pulse, (unsigned long)g_motor[i].pwm_period,
                           g_motor[i].mode == MOTOR_MODE_SPEED ? "SPEED" : "OPEN");
        }
        return;
    }

    /* ================================================================ */
    /* PWMSCAN: 占空比扫描找电机死区 (阻塞式调试命令)                    */
    /*   格式: PWMSCAN,<motor>,<start>,<end>,<step>,<hold_ms>            */
    /*   例:   PWMSCAN,1,0.05,0.4,0.05,500                               */
    /*         → 电机1从5%扫到40%, 每次5%, 每步保持500ms                  */
    /*   例:   PWMSCAN,ALL,0.1,0.3,0.05,300                              */
    /*         → 四轮同时扫描                                            */
    /* ================================================================ */
    if (strncmp(cmd, "PWMSCAN,", 8) == 0) {
        char motor_str[8] = {0};
        float start_duty, end_duty, step;
        int hold_ms;
        int n = sscanf(cmd, "%*[^,],%7[^,],%f,%f,%f,%d",
                       motor_str, &start_duty, &end_duty, &step, &hold_ms);
        if (n < 5) {
            UART_IO_Printf("ERR PWMSCAN,<motor|ALL>,<start>,<end>,<step>,<hold_ms>\r\n");
            UART_IO_Printf("ERR  e.g. PWMSCAN,1,0.05,0.4,0.05,500\r\n");
            return;
        }
        if (hold_ms < 100)  hold_ms = 100;
        if (hold_ms > 5000) hold_ms = 5000;
        if (start_duty < 0.0f) start_duty = 0.0f;
        if (end_duty   > 1.0f) end_duty   = 1.0f;
        if (step <= 0.0f) step = 0.05f;
        if (start_duty >= end_duty) {
            UART_IO_Printf("ERR start must be < end\r\n");
            return;
        }

        /* 解析电机ID */
        int motor_id = -1;  /* -1=ALL */
        if (strcmp(motor_str, "ALL") == 0 || strcmp(motor_str, "all") == 0) {
            motor_id = -1;
        } else {
            motor_id = atoi(motor_str) - 1;
            if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
                UART_IO_Printf("ERR motor must be 1-4 or ALL\r\n");
                return;
            }
        }

        UART_IO_Printf("=== PWM Scan %s: %.2f → %.2f step=%.2f hold=%dms ===\r\n",
                       (motor_id < 0) ? "ALL" : 
                       (motor_id == 0 ? "M1" : motor_id == 1 ? "M2" : 
                        motor_id == 2 ? "M3" : "M4"),
                       start_duty, end_duty, step, hold_ms);

        /* 先停所有电机 */
        Motor_StopAll();

        int total_steps = (int)((end_duty - start_duty) / step + 0.5f) + 1;
        float duty = start_duty;

        for (int s = 0; s < total_steps && duty <= end_duty + 0.0001f; s++, duty += step) {
            if (duty > end_duty) duty = end_duty;

            /* 设置占空比 */
            if (motor_id < 0) {
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    Motor_SetDuty((uint8_t)i, duty);
                }
            } else {
                Motor_SetDuty((uint8_t)motor_id, duty);
            }

            /* 等待电机响应 (分小段等, 避免阻塞太久) */
            for (int w = 0; w < hold_ms / 10; w++) {
                HAL_Delay(10);
                /* 每 100ms 刷新看门狗/处理串口 */
                if (w % 10 == 9) {
                    while (UART_IO_Available() > 0) {
                        uint8_t ch;
                        if (UART_IO_GetChar(&ch) && (ch == '\r' || ch == '\n')) {
                            /* 检测到回车 → 中止扫描 */
                            Motor_StopAll();
                            UART_IO_Printf("\r\nABORTED by user\r\n");
                            return;
                        }
                    }
                }
                Motor_UpdateAll(0.01f);
            }

            /* 读取编码器并输出 */
            if (motor_id < 0) {
                UART_IO_Printf("DUTY=%.2f | ", duty);
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    UART_IO_Printf("M%d:%.0fRPM%s", i + 1, (double)g_motor[i].speed_rpm,
                                   (i < MOTOR_COUNT - 1) ? " " : "");
                }
                UART_IO_Printf("\r\n");
            } else {
                UART_IO_Printf("DUTY=%.2f | M%d: delta=%ld cnt, speed=%.0f RPM, pulse=%lu\r\n",
                               duty, motor_id + 1,
                               (long)g_motor[motor_id].count_delta,
                               (double)g_motor[motor_id].speed_rpm,
                               (unsigned long)__HAL_TIM_GET_COMPARE(
                                   g_motor[motor_id].htim_pwm,
                                   g_motor[motor_id].pwm_channel));
            }
        }

        Motor_StopAll();
        UART_IO_Printf("=== Scan complete ===\r\n");
        return;
    }

    /* ================================================================ */
    /* PWM: 直接指定 pulse 值 (0~99), 正=前进 负=后退                     */
    /*   格式: PWM,<motor|ALL>,<pulse>                                   */
    /*   例:   PWM,1,20    → 电机1 pulse=20/99 前进 (~20%)               */
    /*         PWM,1,-15   → 电机1 pulse=15/99 后退 (~15%)               */
    /*         PWM,ALL,30  → 四轮 pulse=30/99 前进 (~30%)               */
    /* ================================================================ */
    if (strncmp(cmd, "PWM,", 4) == 0) {
        char motor_str[8] = {0};
        int pulse;
        int n = sscanf(cmd, "%*[^,],%7[^,],%d", motor_str, &pulse);
        if (n != 2) {
            UART_IO_Printf("ERR PWM,<motor|ALL>,<pulse>  e.g. PWM,1,20\r\n");
            return;
        }
        int dir = (pulse >= 0) ? 1 : -1;
        int abs_pulse = (pulse >= 0) ? pulse : -pulse;
        if (abs_pulse > 99) abs_pulse = 99;

        int motor_id = -1;
        if (strcmp(motor_str, "ALL") == 0 || strcmp(motor_str, "all") == 0) {
            motor_id = -1;
        } else {
            motor_id = atoi(motor_str) - 1;
            if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
                UART_IO_Printf("ERR motor must be 1-4 or ALL\r\n");
                return;
            }
        }

        const char *dir_str = (dir >= 0) ? "FWD" : "REV";

        if (motor_id < 0) {
            Motor_StopAll();
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (dir >= 0) {
                    HAL_GPIO_WritePin(g_motor[i].in1_port, g_motor[i].in1_pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(g_motor[i].in2_port, g_motor[i].in2_pin, GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(g_motor[i].in1_port, g_motor[i].in1_pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(g_motor[i].in2_port, g_motor[i].in2_pin, GPIO_PIN_SET);
                }
                __HAL_TIM_SET_COMPARE(g_motor[i].htim_pwm, g_motor[i].pwm_channel, (uint32_t)abs_pulse);
            }
            UART_IO_Printf("OK PWM ALL=%s pulse=%d/99 (%.0f%%)\r\n",
                           dir_str, abs_pulse, abs_pulse * 100.0f / 99.0f);
        } else {
            Motor_Stop((uint8_t)motor_id);
            if (dir >= 0) {
                HAL_GPIO_WritePin(g_motor[motor_id].in1_port, g_motor[motor_id].in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(g_motor[motor_id].in2_port, g_motor[motor_id].in2_pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(g_motor[motor_id].in1_port, g_motor[motor_id].in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(g_motor[motor_id].in2_port, g_motor[motor_id].in2_pin, GPIO_PIN_SET);
            }
            __HAL_TIM_SET_COMPARE(g_motor[motor_id].htim_pwm, g_motor[motor_id].pwm_channel, (uint32_t)abs_pulse);
            UART_IO_Printf("OK PWM M%d=%s pulse=%d/99 (%.0f%%)\r\n",
                           motor_id + 1, dir_str, abs_pulse, abs_pulse * 100.0f / 99.0f);
        }
        return;
    }

    /* ================================================================ */
    /* PIDTUNE: 在线调整电机 PID 参数 (无须重新编译)                      */
    /*   格式: PIDTUNE,<motor|ALL>,<Kp>,<Ki>,<Kd>                        */
    /*   例:   PIDTUNE,ALL,0.001,0.015,0.00001                           */
    /*         PIDTUNE,1,0.002,0.02,0.00001                              */
    /*   PIDTUNE,?    → 查看当前参数                                      */
    /* ================================================================ */
    if (strncmp(cmd, "PIDTUNE", 7) == 0) {
        if (cmd[7] == ',' || (cmd[7] == '?' && cmd[8] == '\0')) {
            if (cmd[7] == '?' && cmd[8] == '\0') {
                /* 查询当前 PID 参数 */
                UART_IO_Printf("=== Current PID Parameters ===\r\n");
                for (int i = 0; i < 4; i++) {
                    UART_IO_Printf("M%d: Kp=%.5f Ki=%.4f Kd=%.6f\r\n", i+1,
                                   (double)g_motor[i].pid_speed.Kp,
                                   (double)g_motor[i].pid_speed.Ki,
                                   (double)g_motor[i].pid_speed.Kd);
                }
                return;
            }
            char motor_str[8] = {0};
            float Kp, Ki, Kd;
            int n = sscanf(cmd, "%*[^,],%7[^,],%f,%f,%f", motor_str, &Kp, &Ki, &Kd);
            if (n != 4) {
                UART_IO_Printf("ERR PIDTUNE,<motor|ALL>,<Kp>,<Ki>,<Kd>\r\n");
                UART_IO_Printf("ERR  e.g. PIDTUNE,1,0.002,0.02,0.00001\r\n");
                return;
            }
            int motor_id = -1;
            if (strcmp(motor_str, "ALL") == 0 || strcmp(motor_str, "all") == 0) {
                motor_id = -1;
            } else {
                motor_id = atoi(motor_str) - 1;
                if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
                    UART_IO_Printf("ERR motor must be 1-4 or ALL\r\n");
                    return;
                }
            }
            if (motor_id < 0) {
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    PID_SetTunings(&g_motor[i].pid_speed, Kp, Ki, Kd);
                }
                UART_IO_Printf("OK PIDTUNE ALL Kp=%.5f Ki=%.4f Kd=%.6f\r\n", Kp, Ki, Kd);
            } else {
                PID_SetTunings(&g_motor[motor_id].pid_speed, Kp, Ki, Kd);
                UART_IO_Printf("OK PIDTUNE M%d Kp=%.5f Ki=%.4f Kd=%.6f\r\n", motor_id+1, Kp, Ki, Kd);
            }
            return;
        }
    }

    /* ================================================================ */
    /* 简单方向控制: FWD / BACK / LEFT / RIGHT (PID 速度闭环)            */
    /* ================================================================ */
    if (strncmp(cmd, "FWD,", 4) == 0) {
        float speed;
        if (sscanf(cmd, "%*[^,],%f", &speed) == 1 && speed > 0) {
            Chassis_MoveManual(speed, 0.0f);
            UART_IO_Printf("OK FWD %.2f m/s\r\n", speed);
        } else {
            UART_IO_Printf("ERR FWD,<speed_mps>  e.g. FWD,0.5\r\n");
        }
        return;
    }

    if (strncmp(cmd, "BACK,", 5) == 0) {
        float speed;
        if (sscanf(cmd, "%*[^,],%f", &speed) == 1 && speed > 0) {
            Chassis_MoveManual(-speed, 0.0f);
            UART_IO_Printf("OK BACK %.2f m/s\r\n", speed);
        } else {
            UART_IO_Printf("ERR BACK,<speed_mps>  e.g. BACK,0.3\r\n");
        }
        return;
    }

    if (strncmp(cmd, "LEFT,", 5) == 0) {
        float angular;
        if (sscanf(cmd, "%*[^,],%f", &angular) == 1 && angular > 0) {
            Chassis_MoveManual(0.0f, angular);
            UART_IO_Printf("OK LEFT %.1f deg/s\r\n", angular);
        } else {
            UART_IO_Printf("ERR LEFT,<deg_per_sec>  e.g. LEFT,90\r\n");
        }
        return;
    }

    if (strncmp(cmd, "RIGHT,", 6) == 0) {
        float angular;
        if (sscanf(cmd, "%*[^,],%f", &angular) == 1 && angular > 0) {
            Chassis_MoveManual(0.0f, -angular);
            UART_IO_Printf("OK RIGHT %.1f deg/s\r\n", angular);
        } else {
            UART_IO_Printf("ERR RIGHT,<deg_per_sec>  e.g. RIGHT,90\r\n");
        }
        return;
    }

    /* ================================================================ */
    /* 开环方向控制 (不用 PID, 纯占空比):                                */
    /*   DUTY_FWD,<0..1>   DUTY_BACK,<0..1>                             */
    /*   DUTY_LEFT,<0..1>  DUTY_RIGHT,<0..1>                            */
    /* 用于诊断 PID/编码器是否引起振荡                                    */
    /* ================================================================ */
    if (strncmp(cmd, "DUTY_FWD,", 9) == 0) {
        float duty;
        if (sscanf(cmd, "%*[^,],%f", &duty) == 1 && duty > 0 && duty <= 1.0f) {
            Chassis_Stop();  /* 关闭底盘状态机, 防止 SetDiffDrive 覆盖 */
            Motor_StopAll();
            Motor_SetDuty(MOTOR_LEFT_FRONT,  duty);
            Motor_SetDuty(MOTOR_LEFT_REAR,   duty);
            Motor_SetDuty(MOTOR_RIGHT_FRONT, duty);
            Motor_SetDuty(MOTOR_RIGHT_REAR,  duty);
            UART_IO_Printf("OK DUTY_FWD %.2f (OPEN_LOOP, no PID, no encoder)\r\n", duty);
        } else {
            UART_IO_Printf("ERR DUTY_FWD,<0..1>  e.g. DUTY_FWD,0.3\r\n");
        }
        return;
    }

    if (strncmp(cmd, "DUTY_BACK,", 10) == 0) {
        float duty;
        if (sscanf(cmd, "%*[^,],%f", &duty) == 1 && duty > 0 && duty <= 1.0f) {
            Chassis_Stop();
            Motor_StopAll();
            Motor_SetDuty(MOTOR_LEFT_FRONT,  -duty);
            Motor_SetDuty(MOTOR_LEFT_REAR,   -duty);
            Motor_SetDuty(MOTOR_RIGHT_FRONT, -duty);
            Motor_SetDuty(MOTOR_RIGHT_REAR,  -duty);
            UART_IO_Printf("OK DUTY_BACK %.2f (OPEN_LOOP, no PID)\r\n", duty);
        } else {
            UART_IO_Printf("ERR DUTY_BACK,<0..1>  e.g. DUTY_BACK,0.3\r\n");
        }
        return;
    }

    if (strncmp(cmd, "DUTY_LEFT,", 10) == 0) {
        float duty;
        if (sscanf(cmd, "%*[^,],%f", &duty) == 1 && duty > 0 && duty <= 1.0f) {
            Chassis_Stop();
            Motor_StopAll();
            Motor_SetDuty(MOTOR_LEFT_FRONT,  -duty);
            Motor_SetDuty(MOTOR_LEFT_REAR,   -duty);
            Motor_SetDuty(MOTOR_RIGHT_FRONT,  duty);
            Motor_SetDuty(MOTOR_RIGHT_REAR,   duty);
            UART_IO_Printf("OK DUTY_LEFT %.2f (OPEN_LOOP, no PID)\r\n", duty);
        } else {
            UART_IO_Printf("ERR DUTY_LEFT,<0..1>  e.g. DUTY_LEFT,0.3\r\n");
        }
        return;
    }

    if (strncmp(cmd, "DUTY_RIGHT,", 11) == 0) {
        float duty;
        if (sscanf(cmd, "%*[^,],%f", &duty) == 1 && duty > 0 && duty <= 1.0f) {
            Chassis_Stop();
            Motor_StopAll();
            Motor_SetDuty(MOTOR_LEFT_FRONT,   duty);
            Motor_SetDuty(MOTOR_LEFT_REAR,    duty);
            Motor_SetDuty(MOTOR_RIGHT_FRONT, -duty);
            Motor_SetDuty(MOTOR_RIGHT_REAR,  -duty);
            UART_IO_Printf("OK DUTY_RIGHT %.2f (OPEN_LOOP, no PID)\r\n", duty);
        } else {
            UART_IO_Printf("ERR DUTY_RIGHT,<0..1>  e.g. DUTY_RIGHT,0.3\r\n");
        }
        return;
    }

    /* ================================================================ */
    /* 单电机控制: M1 / M2 / M3 / M4 / MALL                             */
    /* ================================================================ */
    if (cmd[0] == 'M') {
        int motor_id = -1;  /* -1=ALL */

        if (strncmp(cmd, "M1,", 3) == 0)      { motor_id = MOTOR_ID_1; cmd += 3; }
        else if (strncmp(cmd, "M2,", 3) == 0) { motor_id = MOTOR_ID_2; cmd += 3; }
        else if (strncmp(cmd, "M3,", 3) == 0) { motor_id = MOTOR_ID_3; cmd += 3; }
        else if (strncmp(cmd, "M4,", 3) == 0) { motor_id = MOTOR_ID_4; cmd += 3; }
        else if (strncmp(cmd, "MALL,", 5) == 0) { motor_id = -1; cmd += 5; }
        else {
            UART_IO_Printf("ERR Unknown motor command: %s\r\n", cmd);
            return;
        }

        /* 判断子命令类型 */
        if (strncmp(cmd, "STOP", 4) == 0) {
            /* Mx,STOP */
            if (motor_id >= 0) {
                Motor_Stop((uint8_t)motor_id);
                UART_IO_Printf("OK M%d STOP\r\n", motor_id + 1);
            } else {
                Motor_StopAll();
                UART_IO_Printf("OK MALL STOP\r\n");
            }
        } else if (strncmp(cmd, "DUTY,", 5) == 0) {
            /* Mx,DUTY,<duty> */
            float duty;
            if (sscanf(cmd, "%*[^,],%f", &duty) == 1) {
                if (motor_id >= 0) {
                    Motor_SetDuty((uint8_t)motor_id, duty);
                    UART_IO_Printf("OK M%d DUTY=%.2f\r\n", motor_id + 1, duty);
                } else {
                    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                        Motor_SetDuty(i, duty);
                    }
                    UART_IO_Printf("OK MALL DUTY=%.2f\r\n", duty);
                }
            } else {
                UART_IO_Printf("ERR Mx,DUTY,<-1..1>\r\n");
            }
        } else {
            /* Mx,<RPM> 速度闭环 */
            float rpm;
            if (sscanf(cmd, "%f", &rpm) == 1) {
                if (motor_id >= 0) {
                    Motor_SetSpeed((uint8_t)motor_id, rpm);
                    UART_IO_Printf("OK M%d RPM=%.0f\r\n", motor_id + 1, rpm);
                } else {
                    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                        Motor_SetSpeed(i, rpm);
                    }
                    UART_IO_Printf("OK MALL RPM=%.0f\r\n", rpm);
                }
            } else {
                UART_IO_Printf("ERR Mx,<RPM>\r\n");
            }
        }
        return;
    }

    /* ================================================================ */
    /* 导航模式: N/E/S/W 或 DIR                                         */
    /* ================================================================ */
    float heading, speed, distance;
    char dir = cmd[0];

    if ((dir == 'N' || dir == 'E' || dir == 'S' || dir == 'W') && cmd[1] == ',') {
        /* 格式: X,<speed>,<distance> */
        if (sscanf(cmd, "%*c,%f,%f", &speed, &distance) != 2) {
            UART_IO_Printf("ERR Format: X,<speed_mps>,<distance_m>\r\n");
            return;
        }
        switch (dir) {
            case 'N': heading = 0.0f;   break;
            case 'E': heading = 90.0f;  break;
            case 'S': heading = 180.0f; break;
            case 'W': heading = 270.0f; break;
            default:  return;
        }
    } else if (strncmp(cmd, "DIR,", 4) == 0) {
        /* 格式: DIR,<heading_deg>,<speed>,<distance> */
        if (sscanf(cmd, "%*[^,],%f,%f,%f", &heading, &speed, &distance) != 3) {
            UART_IO_Printf("ERR Format: DIR,<deg>,<speed>,<distance>\r\n");
            return;
        }
    } else {
        UART_IO_Printf("ERR Unknown: %s\r\n", cmd);
        return;
    }

    /* 参数保护 */
    if (speed <= 0.0f)   speed = 0.5f;
    if (distance <= 0.0f) distance = 0.5f;

    /* 执行导航命令 */
    bool ok = Chassis_GoDirection(heading, speed, distance);
    if (ok) {
        const char *name;
        if (heading < 0.5f || heading > 359.5f)
            name = "North";
        else if (heading > 89.5f && heading < 90.5f)
            name = "East";
        else if (heading > 179.5f && heading < 180.5f)
            name = "South";
        else if (heading > 269.5f && heading < 270.5f)
            name = "West";
        else
            name = "Custom";

        UART_IO_Printf("OK Go: heading=%.0f°(%s) speed=%.2fm/s distance=%.2fm\r\n",
                       heading, name, speed, distance);
    } else {
        UART_IO_Printf("ERR Busy: chassis is executing a task\r\n");
    }
}

/**
 * @brief 上报当前状态到上位机 (使用里程计融合数据)
 */
/* 【中文名】汇报状态：每500ms向上位机发一行 $STAT 状态（状态/航向/坐标/距离/四轮转速） */
static void ReportStatus(void)
{
    const Odom_Pose *pose = Odom_GetPose();   /* 取里程计位姿 */
    float heading   = pose->heading;          /* 航向（度） */
    float traveled  = Chassis_GetTraveledDistance();   /* 已走距离 */
    float remaining = Chassis_GetRemainingDistance();  /* 剩余距离 */

    /* 读四个电机的当前转速 */
    float spd1 = Motor_GetSpeed(MOTOR_ID_1);
    float spd2 = Motor_GetSpeed(MOTOR_ID_2);
    float spd3 = Motor_GetSpeed(MOTOR_ID_3);
    float spd4 = Motor_GetSpeed(MOTOR_ID_4);

    /* 拼成一行 $STAT 发出去（.3f = 小数3位，%.1f = 小数1位，%.0f = 整数） */
    UART_IO_Printf("$STAT,%s,HDG=%.1f,X=%.3f,Y=%.3f,DIST=%.2f,REM=%.2f,"
                   "Vx=%.3f,Vy=%.3f,Wz=%.3f,"
                   "M1=%.0f,M2=%.0f,M3=%.0f,M4=%.0f\r\n",
                   Chassis_GetStateName(), heading,
                   (double)pose->x, (double)pose->y,
                   traveled, remaining,
                   (double)pose->vx, (double)pose->vy, (double)pose->omega,
                   spd1, spd2, spd3, spd4);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 尝试通过串口发送致命错误信息 */
  UART_IO_Print("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
  UART_IO_Printf("!!! FATAL ERROR at tick=%lu ms !!!\r\n", (unsigned long)HAL_GetTick());
  UART_IO_Print("!!! System halted. Check hardware!\r\n");
  UART_IO_Print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");

  /* 等待串口发送完成 */
  for (volatile uint32_t i = 0; i < 1000000; i++) { __NOP(); }

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

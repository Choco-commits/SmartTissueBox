# 智能桌宠纸巾盒（Smart Tissue Box）

基于 STM32F103C8T6 与 FreeRTOS 实时操作系统的智能纸巾盒，实时检测纸巾余量，并根据余量变化切换 OLED 表情与舵机动作，实现拟人化的交互体验。

## 项目简介

基于 STM32F103C8T6 开发板设计一款智能纸巾盒：通过 HC-SR04 超声波测距实时检测纸巾余量，OLED 实时显示余量百分比与表情，双舵机实现「行走庆祝、睡眠提醒、哭泣提醒」等动作；基于 FreeRTOS 多任务调度实现测距、显示、动作、按键各功能并行解耦运行。

## 功能特性

- **余量检测**：HC-SR04 超声波测距，利用定时器输入捕获测量回波脉宽，映射为 0~100% 余量。
- **数据滤波**：滑动中值滤波 + 量化处理 + 量程合理性校验，抑制近距离测量抖动。
- **表情交互**：OLED 显示余量进度条、百分比与表情图案，随余量档位自动切换。
- **拟人化动作**：双舵机 PWM 控制，实现行走庆祝、睡眠提醒、哭泣提醒等动作。
- **状态机**：充足 / 不足 / 即将耗尽三档状态，按键控制启停，重新开机自动恢复当前状态。

## 硬件清单

| 器件 | 型号 / 说明 |
|------|-------------|
| 主控 | STM32F103C8T6（Cortex-M3） |
| 测距 | HC-SR04 超声波传感器 |
| 显示 | 0.96 英寸 OLED（I2C，128×64） |
| 动作 | 舵机 × 2 |
| 输入 | 轻触按键 × 1 |

### 引脚接线

| 外设 | 引脚 | 功能 |
|------|------|------|
| HC-SR04 TRIG | PA0 | 触发脉冲输出 |
| HC-SR04 ECHO | PA8（TIM1_CH1） | 回波输入捕获 |
| OLED SCL | PB6 | I2C 时钟 |
| OLED SDA | PB7 | I2C 数据 |
| 舵机 1 | PA6（TIM3_CH1） | PWM 输出 |
| 舵机 2 | PA7（TIM3_CH2） | PWM 输出 |
| 按键 | PA1 | 开关机（内部上拉，低电平按下） |

## 软件架构

### 任务划分

| 任务 | 优先级 | 职责 |
|------|:---:|------|
| start_task | 0 | 创建队列与各任务后自我删除 |
| control_task | 3 | 状态机决策，分发 OLED / 舵机命令 |
| button_task | 3 | 按键检测，发送开关机信号 |
| detect_task | 2 | 超声波测距、滤波、上报余量 |
| servo_task | 1 | 执行舵机动作 |
| oled_task | 0 | 显示余量与表情 |

### 任务间通信

- **消息队列**：detect_task 在余量档位发生变化时向 control_task 传递余量数据，减少无效刷新。
- **任务通知**：control_task 向 oled_task / servo_task 分发显示与动作命令；button_task 向 control_task 发送开关机信号。

## 目录结构

```
├── Core/                 # 应用与启动代码（HAL 生成 + 用户逻辑）
│   ├── Inc/
│   └── Src/              # main.c、中断服务、msp 配置
├── Drivers/              # STM32 HAL 库 + CMSIS
├── FreeRTOS-Kernel-main/ # FreeRTOS 内核源码
├── my_lib/               # OLED 驱动
├── face/                 # 表情位图素材
├── MDK-ARM/              # Keil 工程文件（.uvprojx）
└── backup/               # 本地备份 / 测试文件（不入版本库）
```

## 编译与烧录

1. 使用 Keil MDK-ARM 5 打开 `MDK-ARM/TissueLevelDetection.uvprojx`。
2. 编译目标：STM32F103C8T6。
3. 通过 ST-Link 等下载器烧录。

## 参考与致谢

本项目使用了以下开源组件与参考资料，在此致谢：

| 组件 | 说明 | 来源 / 作者 | 许可 |
|------|------|-------------|------|
| FreeRTOS Kernel | 实时操作系统内核 V11.1.x | [FreeRTOS/FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) | MIT |
| STM32F1xx HAL / CMSIS | 外设驱动库与内核 | STMicroelectronics / STM32CubeMX 生成 | BSD-3-Clause |
| OLED 驱动（my_lib/） | OLED 图形显示驱动 | 铁头山羊（见 `my_lib/oled.h` 文件头） | — |
| FreeRTOS 移植方法 | 内核移植思路参考 | 正点原子（ALIENTEK）教程 | 学习参考 |

> 引用上述开源代码请保留其原始版权声明与许可文件（如 `Drivers/*/LICENSE.txt`）。

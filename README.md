# RoboMaster C Board · 云台控制 + 板间通信演示

> 基于 RoboMaster C 板标准例程的 STM32F4 控制工程，集成了惯导解算与云台/板间通信控制。
> STM32F4 control project on the RoboMaster C Board — integrates INS estimation with gimbal & inter-board communication control.

## 项目说明

本工程以 **RoboMaster C 板标准例程（大疆官方 C_Board_Standard_Robot）** 为底座，在其上搭建了：

- **云台控制**：双环（位置环 + 速度环）PID 云台控制，含 360° 圆周角差处理；
- **板间通信**：C 板 ↔ A 板双板 CAN 通信协议（`board_com`），实现跨板数据交换；
- **电机 / 外设集成**：电机封装、CAN 滤波收发（`bsp_can`）、PWM / DWT 等底层驱动；
- **惯导解算**：集成第三方四元数 EKF 惯导算法，提供姿态角与陀螺零偏估计。

## ⚠️ 第三方代码与出处声明（重要）

本仓库**并非全部由本人编写**。为尊重原作者、避免学术/署名纠纷，以下模块保留其原始署名，请在使用与引用时一并致谢：

| 模块 | 原作者 | 说明 |
| --- | --- | --- |
| `ins_task` 惯导任务 | **王洪玺**（哈尔滨工程大学 创梦之翼战队，wanghongxi2001@foxmail.com） | 基于四元数扩展卡尔曼滤波（EKF）的姿态解算，含卡方检验、渐消因子等 |
| `Components/Algorithm/QuaternionEKF` | 王洪玺 | 四元数 EKF 核心 |
| `Components/Controller/controller` | 王洪玺 | 控制器框架 |
| `kalman_filter` / `transfer_function` / `user_lib` | 王洪玺 | 滤波 / 传递函数 / 通用工具 |
| `bsp_dwt` / `bsp_PWM` / `BMI088driver` | 王洪玺 | DWT 计时、PWM、BMI088 驱动 |
| 工程底座 `C_Board_Standard_Robot` | **RoboMaster / 大疆创新** | C 板标准例程（HAL + FreeRTOS + 双 CAN 工程模板） |

> 王洪玺的惯导 EKF 算法详见其公开发表的《惯导姿态解算项目》文档（哈尔滨工程大学创梦之翼战队）。本仓库**不重新托管其 PDF 论文**，仅在此注明出处与作者信息；如需引用算法，请溯源至原作者。
> 大疆 RoboMaster C 板例程遵循 DJI 相关开源/使用条款，使用时请遵守。

## 本人贡献

以下模块由本人编写（无第三方署名、为本工程定制部分）：

| 模块 | 内容 |
| --- | --- |
| `Application/gimbal_task` | 云台双环 PID 控制（位置环 + 速度环）、360° 圆周角差 `circle_error` |
| `Application/board_com` | C 板 ↔ A 板双板 CAN 通信协议与数据结构 |
| `Module/motor` | 电机封装与指令下发 |
| `Bsp/bsp_can` | CAN 滤波器配置与收发 |
| `Algorithm/pid` | 自行实现的 PID 控制器 |

## 硬件平台

- 主控：RoboMaster C 板（STM32F4 系列）
- 实时系统：FreeRTOS（CMSIS-RTOS v2）
- 传感器：BMI088（IMU）、IST8310（磁力计）、DR16（遥控器）
- 外设：双路 CAN、SPI、USART、PWM、TIM
- 工具链：STM32CubeIDE / Keil MDK（含 `C_Board_Standard_Robot.ioc`）

## 目录结构

```
C_Board_Standard_Robot.ioc   # CubeMX 工程
Application/                 # gimbal_task（本人）/ ins_task（王洪玺）/ board_com（本人）
Bsp/                         # bsp_can（本人）/ bsp_PWM / bsp_dwt（王洪玺）
Module/                      # motor（本人）
Algorithm/                   # pid（本人）
Components/                  # Algorithm/QuaternionEKF · Controller/controller（王洪玺）
Src/ Inc/                    # HAL + FreeRTOS 生成代码
Drivers/                     # STM32 HAL + CMSIS
Middlewares/Third_Party/     # FreeRTOS 源码
```

## 编译说明

1. 用 STM32CubeIDE 打开 `C_Board_Standard_Robot.ioc` 重新生成代码，或用 `MDK-ARM/` 下 Keil 工程编译。
2. `Drivers/CMSIS` 中的预编译 DSP 库（`*.lib` / `*.a`）按 `.gitignore` 不纳入版本管理；编译前请确保本地 STM32Cube 固件包内含对应 CMSIS-DSP（惯导解算依赖 `arm_math`）。
3. 惯导任务 `ins_task` 在 `Application/` 中由调度器启动。

## 状态

可用于学习「C 板 + 惯导 EKF + 云台双环 PID + 双板 CAN 通信」的整体链路；其中惯导/IMU 算法栈为第三方开源成果，本人工作集中在云台控制与板间通信集成。

---

© 本人编写部分归本人所有；王洪玺编写部分版权归原作者，详见上方出处声明；HAL / FreeRTOS / CMSIS 等组件遵循其各自许可证。

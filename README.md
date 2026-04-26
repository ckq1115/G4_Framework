G4_Framework 主控框架（STM32G473VET6）

## 概述
基于 STM32G473VET6 的 RoboMaster 机器人主控框架，由于时间有限，目前功能并不完善，底层驱动的优化已经接近尾声，后续会考虑加入上层任务相关函数。
框架的设计思路是严格分层，BSP 层对 HAL 进行轻量封装，Device 层实现具体设备的驱动与协议解析，Algorithm 层实现无硬件依赖的算法，Middleware 层进行系统级数据处理与状态汇总，App 层负责业务逻辑与任务调度。Utils 目录提供全局定义与通用工具函数。但目前各层之间还是存在互相调用的情况，正在想办法解决。
大部分代码是移植的老C板/MC02框架，但是对其进行了重构和优化，去掉了冗余的代码，使整体的逻辑更清晰，很多东西我个人认为写的还是可以的，可以借鉴一下，后续会继续完善功能，增加更多的设备支持和算法实现。

我比较懒，下面这些是AI写的，介绍了文件结构，可以作为一个参考

## 开发环境
- MCU: STM32G473VET6
- IDE: CLion + CMake
- RTOS: FreeRTOS
- Toolchain: GCC ARM None EABI
- Debug: OpenOCD + GDB + Ozone

## 目录结构

### 根目录配置
```
G4_Framework/
├── CMakeLists.txt
├── CMakePresets.json
├── G4_Framework.ioc
├── STM32G473XX_FLASH.ld
├── startup_stm32g473xx.s
├── cmake/
├── Core/
├── Drivers/
├── Middlewares/
├── User/
└── README.md
```

### Core - HAL 层
由 STM32CubeMX 生成的 HAL 初始化、时钟配置、中断处理和系统启动相关代码。

```
Core/
├── Inc/
└── Src/
```

### Drivers - 驱动库
ARM CMSIS 接口与 STM32G4 HAL 驱动库。

```
Drivers/
├── CMSIS/
└── STM32G4xx_HAL_Driver/
```

### Middlewares - 中间件层
包含 ST 与第三方中间件目录（如 FreeRTOS）。

```
Middlewares/
├── ST/
└── Third_Party/
```

---

### User - 应用代码（主要开发区）
User 目录按模块分层组织，便于维护和扩展。

#### 1) Algorithm 算法层
无硬件依赖的算法实现，便于移植。

```
User/Algorithm/
├── Inc/
│   ├── Chassis_Calc.h                 # 底盘计算接口
│   ├── controller.h                   # 控制算法接口（如 PID）
│   ├── kalman_filter.h                # 卡尔曼滤波接口
│   ├── mahony_filter.h                # Mahony 姿态滤波接口
│   ├── Power_Ctrl.h                   # 功率控制接口
│   └── QuaternionEKF.h                # 四元数 EKF 接口
└── Src/
    ├── Chassis_Calc.c                 # 底盘计算实现
    ├── controller.c                   # 控制算法实现
    ├── kalman_filter.c                # 卡尔曼滤波实现
    ├── mahony_filter.c                # Mahony 姿态滤波实现
    ├── Power_Ctrl.c                   # 功率控制实现
    └── QuaternionEKF.c                # 四元数 EKF 实现
```

---

#### 2) Utils 工具层
通用工具与全局定义。

```
User/Utils/
├── Inc/
│   ├── All_define.h                   # 全局宏与类型定义
│   ├── All_Init.h                     # 系统初始化接口
│   ├── All_Motor.h                    # 统一电机接口
│   ├── CKQ_MATH.h                     # 数学库接口
│   ├── CRC_DJI.h                      # DJI CRC 校验接口
│   ├── user_lib.h                     # 通用工具库接口
│   └── Vofa.h                         # Vofa+ 通讯接口
└── Src/
    ├── All_Init.c                     # 系统初始化实现
    ├── CKQ_MATH.c                     # 数学库实现
    ├── CRC_DJI.c                      # DJI CRC 校验实现
    ├── user_lib.c                     # 通用工具库实现
    └── Vofa.c                         # Vofa+ 通讯实现
```

---

#### 3) BSP 板级支持层
对 HAL 的轻量封装，提供更易用的硬件访问接口。

```
User/BSP/
├── Inc/
│   ├── BSP-FDCAN.h                    # FDCAN
│   ├── BSP_DWT.h                      # DWT
│   ├── BSP_QSPI.h                     # QSPI
│   ├── BSP_SPI.h                      # SPI
│   └── WS2812.h                       # WS2812
└── Src/
    ├── BSP_DWT.c                      # DWT 计时
    ├── BSP_FDCAN.c                    # FDCAN 板级实现
    ├── BSP_QSPI.c                     # QSPI 封装库
    ├── BSP_SPI.c                      # SPI 封装库
    └── WS2812.c                       # WS2812 封装库
```

---

#### 4) Device 设备层
外设与模块驱动、协议解析与控制接口。

```
User/Device/
├── Inc/
│   ├── DBUS.h                         # 遥控接收接口
│   ├── DJI_Motor.h                    # DJI 电机接口
│   ├── DM_Motor.h                     # 达妙电机接口
│   ├── ICM42688P.h                    # ICM42688P 设备接口
│   ├── LK_Motor.h                     # 瓴控电机接口
│   ├── Power_CAP.h                    # 超级电容接口
│   ├── Referee.h                      # 裁判系统接口
│   ├── serial_servo.h                 # 串口舵机接口
│   ├── VT13.h                         # VT13 设备接口
│   └── W25N01GV.h                     # W25N01GV Flash 接口
└── Src/
    ├── DBUS.c                         # DBUS接收
    ├── DJI_Motor.c                    # DJI 电机驱动库
    ├── DM_Motor.c                     # 达妙电机库
    ├── ICM42688P.c                    # ICM42688P驱动库
    ├── LK_Motor.c                     # 瓴控电机库
    ├── Power_CAP.c                    # 超级电容通讯
    ├── Referee.c                      # 裁判系统解算
    ├── serial_servo.c                 # 串口舵机实现
    ├── VT13.c                         # 图传链路接收
    └── W25N01GV.c                     # W25N01GV Flash驱动库
```

---

#### 5) Middleware 中间件层
系统级数据处理与状态汇总。

```
User/Middleware/
├── Inc/
│   ├── CAN_Comm.h                     # CAN 通讯与消息定义
│   └── System_Status.h                # 系统状态机接口
└── Src/
    └── System_Status.c                # 系统状态机实现
```

---

#### 6) App 应用层
业务逻辑与任务调度层。

```
User/App/
├── Inc/
│   ├── All_Task.h                     # 任务调度与系统状态相关定义
│   ├── Boomerang_Task.h               # 飞镖任务接口
│   ├── Chassis_Task.h                 # 底盘任务接口
│   ├── IMU_Task.h                     # IMU 任务接口
│   └── Test_Task.h                    # 测试任务接口
└── Src/
    ├── All_Task.c                     # 任务创建与调度实现
    ├── Boomerang_Task.c               # 飞镖任务实现
    ├── Chassis_Task.c                 # 底盘任务实现
    ├── IMU_Task.c                     # IMU 任务实现
    └── Test_Task.c                    # 测试任务实现
```

---

#### 7) UI 界面层
基于 RoboMaster 裁判系统的图形化 UI 实现。

```
User/UI/
├── lib/                               # UI 自动生成库及接口
│   ├── UI.h
│   ├── ui_g1.c
│   ├── ui_g1.h
│   ├── ui_g2.c
│   ├── ui_g2.h
│   ├── ui_interface.c
│   ├── ui_interface.h
│   └── ui_types.h
├── RM_UI_Adapter.c                    # UI 适配器实现
├── RM_UI_Adapter.h                    # UI 适配器接口
├── RMUI.c                             # UI 主要逻辑实现
├── RMUI.h                             # UI 主要逻辑接口
├── ui1.json
├── ui1.rmui
├── ui2.json
└── ui2.rmui
```

## 分层依赖关系
```
App
 ↓ 使用
Device + Algorithm + Middleware
 ↓ 使用
BSP
 ↓ 使用
HAL
```

Utils 可被各层调用。
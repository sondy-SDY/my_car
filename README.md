# STM32 寻迹小车

基于 STM32F103C8Tx 的六路红外寻迹小车项目。项目使用 STM32 HAL 库和 CMake 构建，通过红外传感器识别黑线位置，结合 PID 控制和差速转向实现直道、普通弯道、急弯以及丢线后的自动找线。

## 项目特点

- 使用 6 路数字红外循迹传感器，支持加权误差计算
- 红外输入采用过采样、积分迟滞和连续确认，降低误判
- 使用 PID 控制左右轮差速，D 项带一阶低通滤波
- 根据偏差大小动态调整基础速度，急弯自动降速
- 丢线后分阶段处理：短时间差速滑行、原地搜索、加速扩大搜索范围
- 使用 TIM3 输出双路 PWM，GPIO 控制电机方向
- 支持 VS Code、CMake、Ninja、OpenOCD 开发和烧录流程

## 硬件连接

### 主控

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103C8Tx |
| 时钟 | HSE 外部晶振，PLL 倍频 |
| 开发方式 | STM32 HAL / CMake |
| 烧录调试 | ST-Link + OpenOCD |

### 循迹传感器

传感器输出为数字量，高电平表示检测到黑线。

| 传感器 | 引脚 | 位置 |
| --- | --- | --- |
| S1 | PB12 | 最左 |
| S2 | PB13 | 左侧 |
| S3 | PB14 | 中左 |
| S4 | PB15 | 中右 |
| S5 | PA8 | 右侧 |
| S6 | PA9 | 最右 |

### 电机驱动

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| 左电机 PWM | PA6 / TIM3_CH1 | 左轮调速 |
| 右电机 PWM | PA7 / TIM3_CH2 | 右轮调速 |
| 左电机 IN1 | PB0 | 方向控制 |
| 左电机 IN2 | PB1 | 方向控制 |
| 右电机 IN1 | PB10 | 方向控制 |
| 右电机 IN2 | PB11 | 方向控制 |

PWM 计数周期为 `999`，软件中速度范围为 `-999 ~ 999`。正负号表示电机方向，绝对值表示占空比大小。

## 软件结构

```text
.
├── Core
│   ├── Inc
│   │   ├── main.h
│   │   ├── Trace.h      # 循迹传感器定义和状态接口
│   │   ├── Motor.h      # 电机控制接口和速度参数
│   │   └── PID.h        # PID 参数和接口
│   └── Src
│       ├── main.c       # 主循环、速度策略、丢线处理
│       ├── Trace.c      # 传感器采样、滤波、误差计算
│       ├── Motor.c      # PWM 输出和电机方向控制
│       └── PID.c        # PID 控制器
├── Drivers              # STM32 HAL / CMSIS 驱动
├── cmake                # CMake 工具链和 CubeMX 构建配置
├── VS_Led_test.ioc      # STM32CubeMX 工程配置
├── openocd.cfg          # OpenOCD 烧录配置
├── flash.bat            # Windows 下 OpenOCD 烧录脚本
└── CMakePresets.json    # Debug / Release 构建预设
```

## 控制逻辑

主循环周期约为 5 ms。每次循环先读取循迹误差，然后根据当前状态决定电机输出。

1. `Trace.c` 读取 6 路传感器，经过多次采样和迟滞确认得到稳定掩码。
2. 根据传感器位置权重 `-5, -3, -1, 1, 3, 5` 计算黑线偏差。
3. `PID.c` 根据偏差输出差速修正量。
4. `main.c` 根据偏差大小动态调整基础速度，并将 PID 输出叠加到左右电机。
5. `Motor.c` 对 PWM 进行限幅和斜率限制，最后输出到电机驱动。

当检测到丢线时，程序不会立即停车，而是按以下策略找线：

1. 短时间沿用丢线前的误差进行差速滑行；
2. 仍未找回黑线时，根据最后的偏移方向原地旋转；
3. 长时间未恢复时提高旋转速度，扩大搜索范围。

## 关键参数

### PID 参数

位于 `Core/Inc/PID.h`：

```c
#define KP    38.0f
#define KI    0.0f
#define KD    50.0f
#define MAX_I 60.0f
#define PID_OUTPUT_LIMIT 420.0f
```

### 电机速度参数

位于 `Core/Inc/Motor.h`：

```c
#define PWM_MAX       999
#define MAX_RUN_SPEED 440
#define TURN_SLOWDOWN 30
#define MOTOR_PWM_STEP 300
#define LEFT_TRIM     0
#define RIGHT_TRIM    0
```

如果小车在直道上明显偏向一侧，可以优先微调 `LEFT_TRIM` 和 `RIGHT_TRIM`。如果过弯冲出黑线，可以适当降低 `MAX_RUN_SPEED` 或提高 `TURN_SLOWDOWN`。

### 循迹滤波参数

位于 `Core/Inc/Trace.h`，包括过采样次数、滤波阈值、丢线保持周期、中心保持周期等。赛道反光、纸面褶皱或传感器安装高度变化较大时，可以从这些参数开始调整。

## 编译环境

建议环境：

- VS Code
- CMake 3.22 或更高版本
- Ninja
- Arm GNU Toolchain，提供 `arm-none-eabi-gcc`
- OpenOCD
- ST-Link 驱动

需要确保 `arm-none-eabi-gcc`、`cmake`、`ninja` 和 `openocd` 已加入系统 PATH。

## 编译

Debug 构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```bash
cmake --preset Release
cmake --build --preset Release
```

构建完成后会生成 ELF 文件，并在构建后自动生成 HEX 文件。

常见输出路径：

```text
build/Debug/VS_Led_test.elf
build/Debug/VS_Led_test.hex
build/Release/VS_Led_test.elf
build/Release/VS_Led_test.hex
```

## 烧录

连接 ST-Link 后，可以在 Windows 下运行：

```bat
flash.bat
```

也可以直接使用 OpenOCD：

```bash
openocd -f openocd.cfg -c "program build/Debug/VS_Led_test.elf verify reset exit"
```

如果使用 Release 固件，请把路径改为：

```bash
openocd -f openocd.cfg -c "program build/Release/VS_Led_test.elf verify reset exit"
```

## 调试和调参建议

- 先架空车轮，确认左右电机方向是否正确。
- 如果小车前进时左右轮方向相反，检查电机驱动 IN1/IN2 接线或 `Motor.c` 中的方向逻辑。
- 确认传感器高电平是否代表黑线；如果模块输出逻辑相反，需要调整 `Trace.c` 中的读取判断。
- 先用低速参数测试直道，再逐步提高 `MAX_RUN_SPEED`。
- 弯道容易冲出时，优先降低急弯速度或增大转弯降速。
- 直道左右偏移时，优先调整 `LEFT_TRIM`、`RIGHT_TRIM`，再调整 PID。

## 许可证

本项目包含 STMicroelectronics 提供的 HAL / CMSIS 驱动文件，相关许可证见 `Drivers` 目录下的 `LICENSE.txt`。项目中用户编写的代码如需开源发布，建议根据实际需求补充独立的开源许可证文件。

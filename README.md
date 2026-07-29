# DC Motor Car

**Version:** 1.4.0-dev

## 项目说明

基于 TI MSPM0G3507 的 **智能小车控制工程**，采用 **app-bsp-middleware-core 四层架构**。

当前 1.4-dev 分支已实现：
- **JY62 陀螺仪驱动** — 六轴姿态数据 UART 状态机解析（角速度/角度/加速度）
- **陀螺仪航向锁** — 比例纠偏控制器（deadband + 限幅）
- **陀螺仪任务调度** — 原地转向 + 距离驱动状态机
- **DMA 循环接收** — UART2 + DMA 乒乓缓冲，零中断逐字回调
- **巡线算法** — 非线性 PD 巡线控制（查表法衰减 + 差速转向）
- **灰度传感器** — 8 路数字灰度传感器读取（多路复用 + GPIO 直接读取）

- **BLE 桥接** — JDY-16 透传模块，MCU 自定义指令集（!ENC 远程采样 + CSV 保存）
- **UART 通信** — 环形缓冲 + 中断收发
- **PI 速度闭环** — 增量式离散 PI 控制器（deadband + 低通滤波）
- **编码器测速** — 双路正交编码器 2x 解码（GPIO 中断 + ISR 内实时计数）
- **电机驱动** — TB6612 双路 H 桥 PWM 控制（正反转 + 占空比）
- **按键控制** — 单击检测状态机
- **OLED 显示** — SSD1306 驱动（软件 SPI，含 6x12 / 8x16 ASCII 字库），显示左右轮目标/实际速度
- **LED 指示** — GPIO 控制（亮、灭、翻转、闪烁）
- **10ms 定时器** — 周期性定时器 ISR（回调模式）
- **延时服务** — SysTick 精密延时（毫秒 / 微秒）
- **四层架构** — app / bsp / middleware / core 分层落地，包含完整编码规范

## 引脚分配 (Pinout)

### 电机驱动 (TB6612)

| 功能 | GPIO | 封装引脚 | 说明 |
|------|------|---------|------|
| PWM A | PB2 | — | TIMA1 CCP0，左轮 PWM |
| PWM B | PB3 | — | TIMA1 CCP1，右轮 PWM |
| AIN1 | PA14 | 7 | 左轮方向控制 1 |
| AIN2 | PA13 | 6 | 左轮方向控制 2 |
| BIN1 | PA16 | 9 | 右轮方向控制 1 |
| BIN2 | PA17 | 10 | 右轮方向控制 2 |

### 舵机 (S20F, 270° 数码舵机)

| 功能 | GPIO | 封装引脚 | 说明 |
|------|------|---------|------|
| PWM 信号 | PA0 | 29 | 50Hz PWM (500~2500μs → 0~270°) |
| 5V | — | — | 外部 5V 供电 |
| GND | — | — | 共地 |

### 编码器

| 功能 | GPIO | 封装引脚 | 说明 |
|------|------|---------|------|
| E1A | PA25 | 26 | 左轮编码器 A 相 |
| E1B | PA26 | 30 | 左轮编码器 B 相 |
| E2A | PB20 | 19 | 右轮编码器 A 相 |
| E2B | PB24 | 23 | 右轮编码器 B 相 |

### 串口

| 外设 | TX | RX | 波特率 | 用途 |
|------|----|----|--------|------|
| UART_1 | PB6 | PB7 | 115200 | K230 视觉模块 / 蓝牙 |
| UART_2 | PB17 | PB16 | 115200 | JY62 陀螺仪 |

### 灰度传感器

| 功能 | GPIO | 封装引脚 | 说明 |
|------|------|---------|------|
| AD0 | PA9 | 55 | 通道选择地址线 bit0 |
| AD1 | PA8 | 54 | 通道选择地址线 bit1 |
| AD2 | PA12 | 5 | 通道选择地址线 bit2 |
| OUT | PA27 | 31 | 数字输出（0=黑, 1=白） |

### OLED (SSD1306, 软件 SPI)

| 功能 | GPIO | 封装引脚 |
|------|------|---------|
| RST | PB14 | 2 |
| DC | PB15 | 3 |
| SCL | PA28 | 35 |
| SDA | PA31 | 39 |

### 其他

| 功能 | GPIO | 封装引脚 | 说明 |
|------|------|---------|------|
| KEY | PA18 | 11 | 用户按键，低电平有效 |
| LED | PB9 | 61 | 用户 LED，高电平点亮 |
| SWCLK | PA20 | — | SWD 调试时钟 |
| SWDIO | PA19 | — | SWD 调试数据 |

## 目录结构

```
├── app/                 # 应用层
│   ├── main.c           # 入口：初始化 + 调度
│   ├── app_control.c    # 应用控制：PI 速度闭环 + 显示更新
│   ├── app_control.h
│   ├── app_line_track_high_speed.c/h      # 高速循线模块
│   ├── app_line_track_low_speed_straight.c/h # 慢速直行循线模块
│   ├── app_line_track_low_speed_circle.c/h   # 慢速环线循线模块
│   ├── app_gyro_task.c  # 陀螺仪任务调度（转向 + 直行）
│   └── app_gyro_task.h
├── bsp/                 # 板级支持包
│   ├── bsp_delay.c/h    # SysTick 延时（ms / us）
│   ├── bsp_led.c/h      # GPIO LED 控制
│   ├── bsp_motor.c/h    # TB6612 电机 PWM 控制（双路 H 桥）
│   ├── bsp_encoder.c/h  # 正交编码器 2x 解码（双路 GPIO 中断）
│   ├── bsp_grayscale.c/h # 8 路灰度传感器（多路复用 + GPIO 读取）
│   ├── bsp_key.c/h      # 按键状态机（单击/双击检测）
│   ├── bsp_timer.c/h    # 10ms 周期定时器（回调模式）
│   ├── bsp_uart.c/h     # UART 环形缓冲 + 中断收发
│   └── bsp_dma_rx.c/h   # UART2 DMA 循环接收
├── middleware/          # 中间件层
│   ├── mid_ble.c/h      # BLE 桥接 + 指令集解析
│   ├── mid_oled.c/h     # SSD1306 OLED 驱动（framebuffer）
│   ├── mid_oledfont.h   # ASCII 字库（6x12 / 8x16）
│   ├── mid_line_track.c/h # 巡线算法（非线性 PD + 查表法）
│   ├── mid_jy62.c/h      # JY62 六轴陀螺仪驱动
│   └── mid_gyro_hold.c/h # 陀螺仪航向保持 PD
├── core/                # 核心层
│   ├── ti_msp_dl_config.c/h  # SysConfig 生成代码
│   ├── startup_mspm0g350x_uvision.s
│   └── DC-Motor-Car.syscfg   # 外设配置源文件
├── tools/               # 工具集
│   ├── keil/            # SysConfig 工具链集成
│   └── ble/             # BLE 桥接命令行工具 + ENC CSV 日志
├── sdk_config.ini       # Windows TI SDK / SysConfig 路径配置
├── apply_sdk_paths.bat  # 根据 sdk_config.ini 更新 .uvprojx 路径
└── DC-Motor-Car.uvprojx  # Keil 项目文件
```

## 分层架构与开发规范

本项目采用 **app / bsp / middleware / core** 四层架构，各层职责和依赖关系如下：

| 层 | 职责 | 可包含的依赖 | 当前模块 |
|----|------|-------------|---------|
| **app/** | 应用逻辑：主循环、状态机、控制算法 | middleware/、bsp/、core/ | `main.c`（入口）、`app_control`（PI 控制 + 显示调度） |
| **middleware/** | 协议/融合层：传感器数据处理、通信协议、算法抽象 | bsp/、core/（`mid_ble` 例外：可引用 `app_control.h`） | `mid_ble`（BLE 桥接 + 指令集）、`mid_oled`（SSD1306 驱动）、`mid_line_track`（巡线算法） |
| **bsp/** | 板级驱动：外设封装（GPIO、UART、PWM、ADC 等） | core/（`ti_msp_dl_config.h`）及标准库 | `bsp_delay`、`bsp_led`、`bsp_motor`、`bsp_encoder`、`bsp_grayscale`、`bsp_key`、`bsp_timer`、`bsp_uart` |
| **core/** | 启动文件、SysConfig 生成代码、链接脚本 | 不包含上层任何文件 | `ti_msp_dl_config.c/h` |

### 关键规范

**1. include 依赖方向 — 单向，禁止反向或跨层循环**

- `bsp/` 模块仅包含 `"ti_msp_dl_config.h"` 和标准库头文件，不包含同层其他 BSP 模块
- `middleware/` 可包含 `bsp/*.h`，不包含 `app/*.h`
- `app/` 可包含 `middleware/*.h` 和 `bsp/*.h`
- 统一使用 `<stdint.h>` 的 `uint8_t`/`uint16_t`/`uint32_t` 等标准类型，不使用自定义别名

**2. 中断所有权 — BSP 拥有 ISR，app 注册回调**

- 外设 ISR 定义在对应的 `bsp/xxx.c` 中，不在 `main.c` 中定义
- BSP 模块提供回调注册接口（如 `Timer_RegisterCallback(func)`）
- `app/main.c` 在初始化时向 BSP 注册回调
- ISR 内仅做：清中断标志 → 调回调 → 设 volatile 状态变量，不做复杂计算

**3. SysConfig 是外设配置的唯一真实来源**

- 所有引脚、外设实例、时钟、DMA 均在 `.syscfg` 中定义
- 应用代码**仅使用**生成的宏（`GPIO_LED_PORT`、`UART_0_INST` 等），绝不硬编码寄存器地址或引脚号
- 新增外设时先修改 `.syscfg`，构建生成 `ti_msp_dl_config.c/h` 后再编写应用代码
- 手写代码绝不直接修改生成的 `ti_msp_dl_config.c/h`

### 蓝牙指令集扩展规范

> 适用于 `middleware/mid_ble.c` 和 `tools/ble/jdy16_ble.py` 的指令集扩展。

**1. 指令格式**

- 指令以 `!` 开头，大写字母，参数空格分隔，以 `\r\n` 结尾
- 未识别指令必须返回 `?CMD\r\n`
- 成功响应：`OK XXX:...\r\n`，失败响应：`ERR XXX:...\r\n`

**2. MCU 端 (`mid_ble.c`) 加指令步骤**

*同步指令*（立即响应，无需后台任务）：
1. 写 `handle_xxx_command(const char *cmd)` 解析参数（`atoi()` 或手动指针），边界限幅
2. 在 `dispatch_command()` 中加前缀匹配分支
3. 无需改动 `MID_BLE_Poll()` 和 `MID_BLE_Init()`

*异步指令*（需要定时轮询，如 `!ENC`）：
1. 声明 `mid_ble_xxx_task_t` 状态结构体 + `static` 实例
2. 写 `handle_xxx_command()` 填充任务参数 + `xxx_task_poll()` 做定时逻辑
3. 在 `dispatch_command()` 加分支
4. 在 `MID_BLE_Poll()` 中调用 `xxx_task_poll()`
5. 在 `MID_BLE_Init()` 中初始化 `s_xxx_task.active = 0`

**3. 绝对禁止**

| 禁止 |
|------|
| 主循环/命令处理中使用 `BSP_Delay_ms()` 阻塞 |
| 直接访问 `app_control.c` 的 `static` 变量（如 `s_motor_left.target_speed`） |
| 需要写操作时先在 `app_control.h` 暴露新的 setter API |
| 读原始编码器计数（会被 10ms 定时器 ISR 清零） → 用 `APP_Control_GetSpeedA/B()` |
| 在 `mid_ble.c` 中重复 SysConfig 配置（波特率等） |
| UART 响应遗漏 `\r\n` 结尾 |
| 单条回复超过 `LINE_BUF_SIZE`（128 字节） |

**4. Python 端 (`jdy16_ble.py`) 同步**

- 新指令需要 PC 端采集/保存数据 → 在 `send_command()` 加触发逻辑，在 `notification_printer()` 加数据行解析
- 所有新增指令必须更新 `print_help()` 和 `tools/ble/README.md`

**5. 架构边界**

`mid_ble.c` 可依赖：
- `bsp_uart` — `BSP_UART_SendString/Byte`、`BSP_UART_Available/ReadByte`
- `bsp_delay` — `BSP_Delay_GetTick()`
- `app_control` — getter/setter API（已暴露到 `.h` 的）

新增 middle 或 bsp 的 `.c` 文件必须手动加入 Keil 项目对应 Group。

---

## 编码规范

### 1. 通用约定

| 项目 | 规则 |
|------|------|
| 缩进 | 4 空格，不使用 Tab |
| 语言标准 | C99 |
| 整数类型 | 统一使用 `<stdint.h>` 标准类型（`uint8_t`、`uint16_t`、`uint32_t`、`int32_t` 等） |
| 布尔类型 | 使用 `<stdbool.h>` 的 `true` / `false` |
| 自定义别名 | 禁止 — 不使用 `u8`、`u16`、`u32` 等非标准缩写 |
| 大括号风格 | K&R 风格（函数定义换行，控制语句不换行） |

### 2. 命名规范

| 类别 | 格式 | 示例 |
|------|------|------|
| 文件名 | `层_模块.c` / `层_模块.h` | `bsp_led.c`, `mid_oled.c`, `app_control.c` |
| 公开函数 | `层_模块_动作()` | `BSP_LED_Init()`, `MID_OLED_DisplayString()` |
| 内部 / `static` 函数 | `模块_动作()`（小写 + 下划线） | `led_set_gpio()`, `uart_rx_isr()` |
| SysConfig 生成的宏 | **直接使用**，不重命名、不 alias | `GPIO_LED_PORT`, `UART_0_INST` |
| 用户自定义宏 | `大写_下划线` | `LINE_SENSOR_THRESHOLD`, `OLED_WIDTH` |
| `static` 全局变量 | `s_` 前缀 | `s_callback`, `s_initialized` |
| 全局变量（极少使用） | `g_` 前缀 | `g_system_state` |
| 回调类型 `typedef` | `层_模块_回调_t` | `bsp_timer_callback_t` |
| 枚举常量 | `大写_下划线` | `CONTROL_STATE_IDLE`, `SENSOR_LEFT` |
| 枚举类型 `typedef` | `层_模块_枚举_t` | `bsp_led_color_t`, `app_state_t` |

### 3. 模块接口规范

每个 BSP 模块遵循统一的接口模式：

| 接口 | 说明 | 是否必须 |
|------|------|---------|
| `BSP_<Module>_Init(void)` | 初始化外设，返回 `bool`（true=成功） | **必须** |
| `BSP_<Module>_RegisterCallback(cb)` | 注册中断回调 | 按需 |
| `BSP_<Module>_Read()` / `_Write()` / `_Control()` | 功能函数 | 按需 |

Middleware 模块前缀改为 `MID_`，App 模块前缀改为 `APP_`。

### 4. 文件组织模板

```c
// ==================== bsp_xxx.h ====================
#ifndef BSP_XXX_H
#define BSP_XXX_H

#include <stdint.h>
#include <stdbool.h>

// Init
bool BSP_Xxx_Init(void);

// Callback registration (if needed)
typedef void (*bsp_xxx_callback_t)(void);
void BSP_Xxx_RegisterCallback(bsp_xxx_callback_t cb);

// Functional API
void BSP_Xxx_Action(void);

#endif
```

```c
// ==================== bsp_xxx.c ====================
#include "bsp_xxx.h"
#include "ti_msp_dl_config.h"

// Internal constants
#define XXX_FOO  100

// Static state
static bsp_xxx_callback_t s_callback = NULL;
static volatile bool s_event_flag = false;

// Internal helper
static void xxx_do_something(void) {
    // ...
}

// Public API
bool BSP_Xxx_Init(void) {
    // DL_ 初始化调用，使用 SysConfig 生成的宏
    return true;
}

void BSP_Xxx_RegisterCallback(bsp_xxx_callback_t cb) {
    s_callback = cb;
}

// ISR (if this module owns an interrupt)
void XXX_IRQHandler(void) {
    // 1. Clear interrupt flag using SysConfig macro
    // 2. Call callback
    // 3. Set volatile state variable
    if (s_callback) {
        s_callback();
    }
}
```

### 5. `main()` 初始化流程

```c
int main(void)
{
    // [1] Core: 时钟、SysTick 等（由 SysConfig 生成）
    SYSCFG_DL_init();

    // [2] BSP 层初始化
    BSP_Delay_Init();
    BSP_LED_Init();
    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_Key_Init();
    BSP_Timer_Init();
    BSP_UART_Init();

    // [3] Middleware 层初始化
    MID_OLED_Init();
    MID_BLE_Init();

    // [4] App 层初始化
    APP_Control_Init();

    // [5] 注册跨层回调
    BSP_Timer_RegisterCallback(on_timer_10ms);
    BSP_Key_RegisterClickCallback(on_key_click);

    while (1) {
        MID_BLE_Poll();
        APP_Control_Run();
    }
}
```

**规则：** `main.c` 只做初始化和主循环调度，不写具体业务逻辑。复杂逻辑拆分到 `app_*.c`。

### 6. ISR 规范

| 规则 | 说明 |
|------|------|
| 归属 | ISR 函数体定义在对应的 `bsp_xxx.c` 中，**不在** `main.c` |
| 内部行为 | **仅做三件事**：清中断标志 → 调用回调 → 设置 `volatile` 状态变量 |
| 复杂计算 | ISR 内不做复杂计算，复杂逻辑在回调或主循环中处理 |
| 回调注册 | BSP 模块提供 `BSP_Xxx_RegisterCallback()` 供 app 层注册 |
| 共享变量 | ISR 和主循环共享的变量必须加 `volatile`，如 `static volatile bool s_flag;` |

> **编码器例外：** `bsp_encoder.c` 的 `GROUP1_IRQHandler` 不遵循回调模式，而是直接在 ISR 内做 2x 正交解码并累加计数，通过 `GetCount` / `ResetCount` API 暴露给 app 层。这是因为编码器脉冲必须在每个边沿实时处理，无法延迟到回调。

#### 回调模式示例

```c
// bsp_timer.h
typedef void (*bsp_timer_callback_t)(void);
void BSP_Timer_RegisterCallback(bsp_timer_callback_t cb);

// bsp_timer.c
static bsp_timer_callback_t s_callback = NULL;
void BSP_Timer_RegisterCallback(bsp_timer_callback_t cb) { s_callback = cb; }
void TIMER_0_IRQHandler(void) {
    DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMER_OVF_INTERRUPT);
    if (s_callback) s_callback();
}

// main.c
static void my_tick_callback(void) { /* app logic */ }
int main(void) {
    SYSCFG_DL_init();
    BSP_Timer_RegisterCallback(my_tick_callback);
    while (1) { __WFE(); }
}
```

### 7. 跨模块依赖细则

```
app/  →  middleware/  →  bsp/  →  core/ (ti_msp_dl_config.h)
 │                        │
 └──── 也可直接引用 bsp/  ┘
```

- **BSP 模块内**：仅包含 `"ti_msp_dl_config.h"` 和标准库头文件（`<stdint.h>`、`<stdbool.h>` 等）
- **BSP 之间**：不互相包含、不互相调用。一个 BSP 模块需要另一个外设配合时，通过 middleware 或 app 层协调
- **Middleware**：可包含 BSP 头文件，不包含 app 头文件
- **App**：可包含 Middleware 和 BSP 头文件

### 8. 延时接口

| 项目 | 内容 |
|------|------|
| 文件 | `bsp/bsp_delay.c` + `bsp/bsp_delay.h` |
| 接口 | `void BSP_Delay_ms(uint32_t ms)`、`void BSP_Delay_us(uint32_t us)` |
| 实现 | 基于 SysTick（1ms 中断 + 寄存器级 µs 补偿），封装在 BSP 内部 |
| 调用方 | 不关心具体定时器实现，直接调 `BSP_Delay_ms()` / `BSP_Delay_us()` 即可 |

### 9. 新文件加入 Keil 项目

新增 `.c` 文件后，必须手动添加到 Keil µVision 项目，否则不会被编译：

1. 在 Project 窗格右键目标 Group（`bsp` / `middleware` / `app`）
2. 选择 **Add Existing Files to Group '\<group\>'**
3. 选择新建的 `.c` 文件
4. `.h` 文件不需要手动添加（编译器通过 Include Path 查找）

> 硬性要求：新增 `.c` 文件后必须执行此操作。

### 10. 注释规范

- **文件头**：保留 TI BSD 3-clause License（与现有 `main.c` 一致）
- **函数头**：简单的行注释说明功能即可，不强制 doxygen
- **关键逻辑**：必须加注释说明意图，避免只有 What 没有 Why

```c
// 好的注释：说明了为什么这么做
if (DL_GPIO_readPins(GPIO_SENSOR_PORT, GPIO_SENSOR_PIN) == 0) {
    // 传感器检测到黑线（低电平有效），切换到寻迹模式
    state = CONTROL_STATE_LINE_FOLLOW;
}
```

### 11. AI 生成代码自检清单

AI（包括本 AI 及后续 vibe coding 中的 AI）完成代码后，**必须逐条自检并如实记录结果**：

```
## 自检清单

- [ ] 文件名 = `层_模块.c/.h` 格式？
- [ ] 公开函数 = `层_模块_动作()` 命名？
- [ ] 静态变量使用 `s_` 前缀？
- [ ] BSP 模块只引用了 `"ti_msp_dl_config.h"` 和标准库？
- [ ] 头文件有 include guard（`#ifndef BSP_XXX_H`）？
- [ ] ISR 定义在 BSP .c 中而非 main.c？
- [ ] ISR 内只做：清标志 → 调回调 → 设 volatile？
- [ ] 外设宏使用 SysConfig 生成的名称而非硬编码？
- [ ] main.c 只做初始化和调度，无业务逻辑？
- [ ] 新 .c 文件已加入 Keil 项目对应 Group？ 没有让用户手动将新文件加入项目？
- [ ] 注释到位？
```

---

## 依赖

- **MCU:** Texas Instruments MSPM0G3507 (Cortex-M0+)
- **SDK:** MSPM0 SDK 2.01.00.03
- **SysConfig:** 1.20.0
- **IDE:** Keil µVision (ARM Compiler V6.22)

## 移植到新机器

1. 安装 TI SDK 和 SysConfig
2. 修改 `sdk_config.ini` 中的实际路径
3. 双击 `apply_sdk_paths.bat` 同步路径到 `.uvprojx`
4. 打开 Keil 编译

## 项目方案初步

### 一、赛题要求

**赛道参数**：环形跑道，AB=CD=1.5m 直线段，BC=DA=R=0.5m 半圆弧，黑线宽 1.8±0.2cm。A 点有 5cm×1.8cm 黑色停止线 + 30cm 校准停止区。

**摆杆规格**：4 根 PPR 水管组成，单根长 25cm，外径 2cm，内壁光滑。内部凹槽放置直径约 1cm 钢珠。摆杆通过合页铰链固定于小车平台，平台高度 h=5cm。摆杆表面标注 0.1cm 刻度线，中心点为 O。

| 序号 | 要求 | 说明 | 满分 |
|------|------|------|------|
| 1 | **图传与记录** | 车载摄像头（固定于摆杆正上方）实时回传画面到场外显示存储设备，录制全程视频，可回放 | 6 |
| 2 | **纯循线行驶** | 小车从 A 点出发，顺时针行驶一整圈，停回 A 点。行驶时间 <20s，停车位置偏差 <2cm | 16 |
| 3 | **静止定点切换** | 小车静止，钢球从中心 O → +5cm 位置稳定 ≥5s，再移动至 -5cm 位置稳定 ≥5s。稳态偏差 <±1cm | 13 |
| 4 | **A→B 动态平衡** | 小车从 A 点出发（钢球置于 O），行驶通过 B 位置。A→B 段行驶时间 <8s，全程钢球稳定在 O 点 ±1cm 内 | 20 |
| 5 | **全程固定点平衡** | 小车从 A 点出发（钢球置于 O），顺时针行驶一整圈通过 A。总时间 <30s，全程钢球稳定在 O 点 ±1cm 内 | 20 |
| 6 | **全程任意点平衡** | 小球从 A 点出发（钢球置于指定刻度位置），顺时针行驶一整圈通过 A。总时间 <30s，全程钢球稳定在指定位置 ±1cm 内 | 20 |
| 7 | **其他** | 方案完整性、实现质量等 | 5 |
| 8 | **设计报告** | 方案论证、理论分析、电路设计、测试结果、报告规范 | 20 |

> **总分 120 分。** 第 4/5/6 项各 20 分，是核心得分项。尺寸约束：整车长宽 ≤ 35cm×25cm。

### 二、整体方案

#### 2.1 机械结构

> 摆杆倾斜调节有两套可选方案：**方案 A（步进+丝杆滑台）** 和 **方案 B（270° 舵机+齿轮齿条）**。两方案的固定端（左侧合页铰链）相同，差异在执行端。

**方案 A：步进电机 + 梯形丝杆滑台**

- **摆杆倾斜调节**：步进电机驱动梯形丝杆直线滑台，滑台滑块垂直升降带动摆杆绕左侧合页铰链做俯仰摆动。
- **固定端**：摆杆左侧合页安装于小车左支撑台（高度 ≥ 5cm），为唯一旋转支点。
- **执行端**：摆杆右端下方垂直布置丝杆滑台，滑块顶端加装缓冲顶片贴合摆杆底面，无间隙同步升降。
- **限位防护**：滑台上下行程设置机械限位挡片 + 程序软限位，防止超程顶翻摆杆。

**方案 B：270° 舵机 + 齿轮齿条**

- **摆杆倾斜调节**：270° 舵机输出轴安装小齿轮（pinion），齿轮与竖直齿条（rack）啮合。舵机旋转驱动齿条竖直升降，齿条顶端顶住摆杆末端底面，带动摆杆绕左侧铰链俯仰。
- **固定端**：与方案 A 相同，左侧合页铰链为唯一旋转支点。
- **执行端**：摆杆右端下方竖直安装齿条+导向槽，齿条顶端贴合摆杆底面。舵机通过支架固定于小车平台，输出轴齿轮与齿条啮合。
- **导向与限位**：齿条在导向槽内竖直滑动，上下行程由机械限位 + 程序软限位保护。
- **重力预紧**：摆杆自重始终压在齿条顶端，齿轮-齿条啮合间隙被单向受力消除，无回差影响。

#### 2.2 运动转换原理

**方案 A（步进+丝杆）：**

步进电机接收 STEP/DIR 脉冲控制信号，驱动丝杆旋转，丝杆螺母副将旋转运动转化为滑块竖直方向直线位移：
- 滑块 **上升** → 摆杆右端抬高 → 凹槽向左倾斜 → 钢球向负方向滑动
- 滑块 **下降** → 摆杆右端下沉 → 凹槽向右倾斜 → 钢球向正方向滑动

通过控制步进电机的脉冲频率与数量，连续调节滑台升降高度，即可精确改变摆杆倾角。

**方案 B（270° 舵机+齿轮齿条）：**

舵机接收 PWM 控制信号，输出轴驱动小齿轮旋转，齿轮-齿条啮合将旋转运动转化为齿条竖直直线位移。映射关系为 **线性**：Δh = r × θ（r 为齿轮分度圆半径，θ 为舵机转角）：
- 齿条 **上升** → 摆杆右端抬高 → 凹槽向左倾斜 → 钢球向负方向滑动
- 齿条 **下降** → 摆杆右端下沉 → 凹槽向右倾斜 → 钢球向正方向滑动

270° 舵机提供比标准 180° 舵机多 50% 的转角行程，在同等齿轮半径下获得更大的竖直行程。以 r = 10mm 齿轮为例，270° 对应齿条行程 ≈ 47mm，覆盖摆杆末端 ±10° 倾角需求。控制接口为单线 PWM，MSPM0G3507 定时器直接输出，无需专用驱动器。

#### 2.3 硬件配套

**方案 A（步进+丝杆）：**

| 单元 | 组成 |
|------|------|
| 动力执行 | 步进电机、梯形丝杆直线滑台、刚性联轴器、滑块顶撑片、合页支撑座 |
| 视觉采集 | K230 视觉模块 + 车载摄像头，实时识别钢球坐标 (cm) |
| 图传 | 车载图传发射模块 + 场外接收显示存储设备 |
| 主控 | TI MSPM0G3507，统一调度循线、步进控制、通信 |
| 传感器 | 8 路灰度传感器（循线）、双路编码器（里程/速度反馈） |
| 驱动 | TB6612 直流电机驱动（小车）、步进电机驱动器（摆杆） |
| 供电 | 车载锂电池统一供电，分路稳压 |

**方案 B（270° 舵机+齿轮齿条）：**

| 单元 | 组成 |
|------|------|
| 动力执行 | 270° 数码舵机（如 MG996R 或同等扭矩型号）、小齿轮（pinion，模数 0.5~1，分度圆半径 ~10mm）、竖直齿条（rack）、齿条导向槽、合页支撑座 |
| 视觉采集 | 同方案 A |
| 图传 | 同方案 A |
| 主控 | 同方案 A（舵机 PWM 由 MSPM0G3507 定时器直接输出，无需专用驱动器） |
| 传感器 | 同方案 A |
| 驱动 | TB6612 直流电机驱动（小车） |
| 供电 | 同方案 A（舵机由 5~6V 供电，需与 MCU 3.3V 分开稳压） |

#### 2.4 软件架构

```
┌─────────────────────────────────────────────────────────┐
│                      app/main.c                         │
│  10ms ISR: 循线控制 + 钢球 PID + 前馈补偿                │
│  main loop: BLE轮询 + K230轮询 + OLED显示               │
└─────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐ ┌─────────────────┐ ┌──────────────────────┐
│ app_line_track  │ │ app_ball_control│ │    app_control       │
│ _high_speed     │ │                 │ │                      │
│ _low_speed      │ │ · 球位置 PID    │ │ · 电机速度 PI        │
│ _straight       │ │ · 加速度前馈    │ │ · BLE 指令分发       │
│ _low_speed      │ │ · 定点状态机    │ │ · 扫频/直驱          │
│ _circle         │ │                 │ │                      │
│ · 循线 PD       │ │                 │ │                      │
│ · 四通道全黑停车│ │                 │ │                      │
│ · PI主动刹车     │ │                 │ │                      │
└────────┬────────┘ └───────┬─────────┘ └──────────┬───────────┘
         │                  │                      │
         ▼                  ▼                      ▼
┌─────────────────┐ ┌─────────────────┐ ┌──────────────────────┐
│ mid_line_track  │ │   mid_k230      │ │      mid_ble         │
│ · 灰度传感器PD  │ │ · 串口帧解析    │ │ · !指令集             │
│ · 差速输出      │ │ · 钢球坐标      │ │                      │
└────────┬────────┘ └───────┬─────────┘ └──────────┬───────────┘
         │                  │                      │
         ▼                  ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│                        BSP 层                               │
│  bsp_motor    bsp_stepper   bsp_encoder   bsp_grayscale     │
│  bsp_uart     bsp_uart2     bsp_timer     bsp_led / key     │
│  bsp_servo (方案B)                                          │
└─────────────────────────────────────────────────────────────┘
```

#### 2.5 闭环控制流程

**方案 A（步进+丝杆）：**

```
K230视觉采集 → UART发送钢球坐标 → MSPM0接收
    ↓
偏差计算: error = ball_x - target_x
    ↓
位置 PID: Kp·error + Kd·derivative + Ki·integral
    ↓
动态前馈: Kff_accel × 车体加速度 + Kff_curve × 弯道曲率
    ↓
输出限幅 → 转换为步进脉冲频率 → STEP/DIR → 步进电机
    ↓
丝杆升降 → 摆杆倾角变化 → 钢球滑动 → K230再次采集 (闭环)
```

**方案 B（270° 舵机+齿轮齿条）：**

```
K230视觉采集 → UART发送钢球坐标 → MSPM0接收
    ↓
偏差计算: error = ball_x - target_x
    ↓
位置 PID: Kp·error + Kd·derivative + Ki·integral
    ↓
动态前馈: Kff_accel × 车体加速度 + Kff_curve × 弯道曲率
    ↓
输出限幅 → 舵角 = PID输出 / (r × 线性系数) → PWM占空比 → 舵机
    ↓
齿轮齿条升降 → 摆杆倾角变化 → 钢球滑动 → K230再次采集 (闭环)
```

> **关键差异**：方案 A 输出为步进脉冲频率（需编码器位置闭环），方案 B 输出为 PWM 占空比（舵机内部闭环，开环给角度指令即可）。外环 PID 结构和参数整定方法两者完全相同。

**前馈补偿来源**：
- 车体纵向加速度 ← 编码器速度差分（`Δspeed / 10ms`）
- 过弯离心力方向 ← 循线误差符号（`MID_LineTrack_GetError()`）

#### 2.6 定点切换逻辑

```
IDLE → MOVE_TO_P5 (target=+5cm) → 到位(|error|<1cm)
     → HOLD_P5 (稳定 ≥5s)
     → MOVE_TO_M5 (target=-5cm) → 到位(|error|<1cm)
     → HOLD_M5 (稳定 ≥5s)
     → DONE (上报总耗时)
```

#### 2.7 待开发模块

| 模块 | 文件 | 职责 | 对应方案 |
|------|------|------|----------|
| 步进电机驱动 | `bsp/bsp_stepper.c/h` | STEP/DIR 脉冲输出，位置追踪 | 方案 A |
| 舵机驱动 | `bsp/bsp_servo.c/h` | PWM 输出，舵机角度控制 | 方案 B |
| K230 串口 | `bsp/bsp_uart2.c/h` | K230 专用 DMA 串口接收 | 通用 |
| K230 协议解析 | `middleware/mid_k230.c/h` | 帧解析，钢球坐标提取 | 通用 |
| 钢球控制 | `app/app_ball_control.c/h` | PID + 前馈 + 定点状态机 |

#### 2.8 K230 ↔ MSPM0 通信协议

- **物理层**：UART，115200bps
- **帧格式**：`"X:+03.5,Y:-01.2\r\n"`（ASCII 文本，坐标单位 cm，精度 0.1cm）
- **发送频率**：20~50ms/帧

### 三、测试方案初步

> 目标：获取系统传递函数关键参数，为 PID 调参提供定量依据。

#### 3.1 测试条件

- K230 稳定输出钢珠坐标
- MSP 端预留 BLE 数据记录链路：`!BALL ON` 开始以 100Hz 发送 `<tick>,<ball_cm>`，`!BALL OFF` 停止
- 方案 A：摆杆倾角通过步进电机脉冲控制
- 方案 B：摆杆倾角通过舵机 PWM 控制（需先完成 3.5 齿轮齿条标定）

#### 3.2 静摩擦死区测试

| 项目 | 内容 |
|------|------|
| **目的** | 找出钢珠从静止开始移动的**最小倾角** θ_min |
| **方法** | 摆杆水平，球置中心 O。倾角从 0° 开始，每次增加约 0.05°~0.1°，保持 3s，观察 K230 坐标是否变化 |
| **判据** | 球开始移动的最小倾角 = 死区上限 |
| **用途** | 死区以下 PID 输出被静摩擦吃掉，积分项持续累积 → windup。用于设定控制死区 + anti-windup 限幅 |

#### 3.3 阶跃响应测试

| 项目 | 内容 |
|------|------|
| **目的** | 获取系统增益 K 和摩擦力等效减速度 |
| **方法** | 球置中心 O。摆杆阶跃到指定倾角 θ，BLE 全程记录 `<tick>,<ball_cm>`（100Hz），直到球滚到 ±12cm 尽头或 3s 超时 |
| **分组** | θ₁ = θ_min + 0.2°（小阶跃）；θ₂ = 0.8°（中等）；θ₃ = 1.5°（大阶跃）。各重复 3 次 |

**后处理**（Python/MATLAB）：

```
1. 位置差分 → 速度 v = gradient(x, 0.01)
               加速度 a = gradient(v, 0.01)
2. 取球启动后前 200ms 的平均加速度 a_mean
3. 系统增益 K = a_mean / θ         (单位: (m/s²)/deg)
4. 摩擦等效减速度 a_fric = g·sin(θ) - a_mean
5. 纯延时 = 阶跃时刻 → 球开始响应的时间差
```

| 输出参数 | 用途 |
|----------|------|
| θ_min | 控制死区下限 + 积分 anti-windup 阈值 |
| K (增益) | 外环 PID 的 Kp 初值估算 |
| a_fric | 评估凹槽顺滑度；若明显偏大（>0.05m/s²），水管内壁可能需要打磨或涂润滑 |
| 纯延时 | K230 帧延迟 + UART 通信延迟的实测值，用于判定外环控制频率上限 |

#### 3.4 后续可选：频率扫描

阶跃响应完成后，可进一步用小幅度正弦扫频（θ = θ_min + A·sin(ωt)，A≈0.3°，ω 从 1Hz 扫到 10Hz），观察钢珠跟踪幅度衰减，确定系统带宽。此步骤为可选优化，不阻塞初步调参。

#### 3.5 齿轮齿条标定（方案 B 专属）

> 目的：建立舵机 PWM 占空比与摆杆末端竖直位移之间的定量关系，为控制算法提供线性映射系数。

| 项目 | 内容 |
|------|------|
| **目的** | 标定 PWM 占空比 → 齿条位移 → 摆杆倾角的线性系数 |
| **方法** | 舵机从 0° 开始，每次增加 10°，用量角器/倾角仪测量摆杆实际倾角，或用卡尺测量齿条顶端高度。记录 PWM 值、舵机转角、齿条高度、摆杆倾角四列数据 |
| **分组** | 0° → 270°，步长 10°，共 28 个采样点。正反行程各 3 次（检测回差） |
| **判据** | ① 线性度 R² > 0.99（齿轮齿条理论上是纯线性）；② 正反行程高度差 < 0.2mm（齿轮啮合回差） |
| **用途** | 拟合斜率 k = Δh/ΔPWM，写入 `bsp_servo.c` 作为 PWM→高度映射常数。外环 PID 输出期望倾角后，直接按线性关系换算为 PWM 值 |

**后处理：**

```
1. 正行程拟合: h_forward = k_forward × PWM + b_forward
2. 反行程拟合: h_backward = k_backward × PWM + b_backward
3. 回差 = mean(|h_forward - h_backward|) 在各 PWM 点的偏差
4. 线性系数 k = (k_forward + k_backward) / 2，写入固件
```

> 齿轮齿条理论映射：Δh = r × θ = r × (270°/PWM_range) × PWM。其中 r 为齿轮分度圆半径，实测拟合的 k 应与理论值一致（偏差 <5%），若偏差过大说明齿轮模数选型有误或啮合不良。

---
## 调参参考 (Tuning Reference)

### 硬件参数

| 参数 | 数值 |
|------|------|
| MCU | TI MSPM0G3507 (Cortex-M0+) |
| 轮径 (Wheel Diameter) | 65mm → 周长 204.2mm |
| 轴距 (Wheelbase) | 0.21m (210mm) |
| 减速比 (Gear Ratio) | 28:1 |
| 编码器 (Encoder) | 500 线 × 2x 倍频 = 1000 脉冲/电机圈 × 28:1 减速比 |
| 灰度传感器 | 8 路数字 (0/1) |
| 控制频率 | 100Hz (10ms 定时器) |
| 电机驱动 | TB6612 双路 H 桥 PWM |

### 编码器校准值 (实测)

| 参数 | 数值 |
|------|------|
| 轮子每圈脉冲数 | **25525** pulses/rev（实测） |
| 每脉冲距离 | 0.008000 mm/pulse |
| 理论值 | 500 × 2 × 28 = 28000（与实际偏差 ~8.8%） |

> 校准方法：上电后在 OLED 上观察 Enc A/B 累计脉冲，手动转动轮子恰好一圈，读取脉冲差值。
> 该值已用于 `app_control.c`、`app_gyro_task.c` 的距离计算。

### 速度闭环 PI (app_line_track.c)

| 参数 | 数值 | 说明 |
|------|------|------|
| KP | 600 | 增量式离散 PI，100Hz |
| KI | 600 | |
| PWM max | ±7800 | |
| Deadband | 0.005 m/s | |

```
pwm += KP × (bias - last_bias) + KI × bias
```

### 灰度循线 PD (mid_line_track.c)

**全局参数：**

| 参数 | 数值 | 说明 |
|------|------|------|
| `MID_TRACK_BASE_SPEED` | 0.35 m/s | 统一直行/弯道速度（无弯道减速） |
| `MID_TRACK_SEARCH_SPEED` | 0.15 m/s | 丢线搜索速度 |
| `MID_TRACK_SEARCH_KP` | 0.038 | 丢线搜索比例增益 |
| `MID_TRACK_SEARCH_CAP` | 6 | 丢线误差限幅 |
| Slew rate limit | ±5/tick | 误差变化率限幅，防止入弯跳变 |

**传感器权重 (s_track_weights):**

```
[-7, -5, -3, -1, 1, 3, 5, 7]
```

**PD 查表 (按 |error| 索引，无衰减表，base_speed 恒定 0.35 m/s)：**

| \|error\| | KP | KD | KD/KP | 转弯半径 R |
|-----------|------|------|--------|-----------|
| 0 | 0.0080 | 0.0047 | 0.59 | ∞ (直行) |
| 1 | 0.0120 | 0.0061 | 0.51 | 3.06m |
| 2 | 0.0180 | 0.0077 | 0.43 | 1.02m |
| 3 | 0.0180 | 0.0077 | 0.43 | 0.68m |
| 4 | 0.0194 | 0.0066 | 0.34 | 0.47m |
| 5 | 0.0178 | 0.0061 | 0.34 | 0.41m |
| 6 | 0.0180 | 0.0077 | 0.43 | 0.34m |
| 7 | 0.0161 | 0.0077 | 0.48 | 0.33m |

> **算法流程：**
>
> 1. 加权平均：`error = Σ(w[i] × sensor[i]) / Σ(sensor[i])`，范围 -7..+7
> 2. Slew rate filter：误差变化每 tick 限幅 ±5，平滑入弯跳变
> 3. `correction = KP[|error|] × error + KD[|error|] × d(error)/dt`
> 4. Correction clamp：`|correction| ≤ base_speed` 防止内侧轮反转
> 5. 差速输出：`left = base_speed + correction`, `right = base_speed - correction`
>
> 转弯半径（稳态）：`R = base_speed × track / (2 × KP × |error|) = 0.0735 / (2 × KP × |error|)`

**丢线行为：**
- 全白 (sum=0) → 向 `s_last_valid_error` 方向搜索，cap 限幅 ±6
- 全黑 (sum=8) → 直行（十字路口策略）

### 舵机 + 齿轮齿条控制参数（方案 B）

#### 零件参数

| 参数 | 符号 | 数值 | 来源 |
|------|------|------|------|
| 齿轮齿数 | z | 25 | 3MF 模型文件 `圆柱齿轮25×1.25.3MF` |
| 齿轮模数 | m | 1.25 | 3MF 模型文件 |
| 分度圆半径 | r | 15.625 mm | r = m × z / 2 |
| 齿条 | — | 模数 1.25，与齿轮匹配 | 3MF 模型文件 `齿条*×1.25.3MF` |
| 舵机型号 | — | 轮趣科技 S20F (270° 数码舵机) | 官方手册 |
| 舵机角度范围 | — | 0 ~ 270° | 官方手册 |
| 舵机角度误差 | — | ≤ 0.54° | 官方手册 |
| PWM 频率 | — | 50 Hz (周期 20ms) | 标准舵机 |
| PWM 脉冲范围 | — | 500 ~ 2500 μs | 官方例程 |
| 舵机信号引脚 | — | PA0 | 参见引脚分配 |
| 铰链到齿条支撑点距离 | L | **待实测**（预估 ~200mm） | 3D 打印组装后卡尺测量 |
| 重力加速度 | g | 9.795 m/s² | 杭州本地实测 |

#### 四级运动学链

##### 第 1 级：PWM → 舵机角度

```
θ_servo = (PWM_us - 500) / 2000 × 270° = (PWM_us - 500) × 0.135  °/μs
```

反向：

```
PWM_us = 500 + θ_servo × 2000/270 = 500 + θ_servo × 7.407  μs/°
```

MSPM0 定时器配置（48MHz 时钟，PSC=47，ARR=19999，1μs 分辨率）：

| 指标 | 值 |
|------|-----|
| 1 PWM 步对应舵机角 | 0.135°/步 |
| 舵机本身误差 0.54° 对应 PWM 步 | 4 步 |
| MSPM0 分辨率 | 远优于舵机本身精度（舵机是精度瓶颈） |

##### 第 2 级：舵机角度 → 齿条竖直位移

齿轮纯滚动，齿条位移与舵机转角严格线性。摆杆自重始终压在齿条顶端，齿轮-齿条啮合间隙被单向受力消除，无回差：

```
h = r × θ_servo(rad)
  = 15.625 × θ_servo(deg) × π/180
  = 0.2727 × θ_servo(deg)  [mm]
```

| θ_servo | h (mm) |
|---------|--------|
| 0.54° (舵机误差) | 0.147 |
| 1° | 0.273 |
| 5° | 1.36 |
| 10° | 2.73 |
| 135° (中位) | 36.8 |
| 270° (满量程) | 73.6 |

##### 第 3 级：齿条位移 → 摆杆倾角

摆杆左端合页铰链固定为唯一支点，右端齿条顶端竖直顶升：

```
tan(φ) = h / L                      （精确公式）
φ(rad) ≈ h / L                       （小角度近似，φ < 10° 时误差 <1%）
φ(deg) = h/L × 180/π = 57.30 × h/L
```

代入 h = 0.2727 × θ_servo：

```
φ(deg) = 15.625 × θ_servo(deg) / L(mm)
```

**机械减速比** = L / 15.625。以 L = 200mm 为例，减速比 **12.8:1**（舵机转 12.8° 摆杆偏 1°）。

| θ_servo (@ L=200mm) | h | φ | 说明 |
|---------------------|------|------|------|
| 0.54° (舵机误差) | 0.147 mm | **0.042°** | 舵机误差经减速后仅 0.042° |
| 1° | 0.273 mm | **0.078°** | |
| 5° | 1.36 mm | **0.39°** | |
| 10° | 2.73 mm | **0.78°** | |

##### 第 4 级：摆杆倾角 → 钢球加速度

钢球在 PPR 水管凹槽内受力（忽略摩擦）：

```
a_ball = g × sin(φ) - a_friction
       ≈ g × φ(rad)        （小角度，sin(φ) ≈ φ）
       = 9.795 × 0.2727 × θ_servo / L
       = 2.671 × θ_servo(deg) / L(mm)  [m/s²]
```

以 L = 200mm 为例：

| θ_servo | a_ball (忽略摩擦) | 说明 |
|---------|-------------------|------|
| 0.54° (误差) | 0.0072 m/s² | 舵机误差导致的加速度扰动，可忽略 |
| 1° | 0.0134 m/s² | |
| 5° | 0.0668 m/s² | |
| 7.5° | **0.100 m/s²** | 有效回复加速度 |
| 10° | 0.134 m/s² | |

#### 关键结论

1. **大减速比容忍舵机误差**：L=200mm 时减速比 12.8:1，舵机 0.54° 误差仅导致摆杆倾角 0.042°、加速度扰动 0.007 m/s²，完全可接受。

2. **舵机工作范围远小于满量程**：产生 0.1 m/s² 回复加速度仅需舵机偏转约 7.5°（摆杆倾角约 0.59°）。舵机绝大部分时间在中位 135°±10° 内工作，仅用了满量程 270° 的约 7%。

3. **弯道离心力是最大扰动源**：v=0.42m/s, R=0.5m 时离心加速度 0.353 m/s²，需补偿舵机偏转约 26°，仍在可用范围内。

4. **二级公式（直接跳过硬计算）**：

```
PWM = 1500 + φ_des(deg) × L(mm) / 2.11
    = 1500 + φ_des(rad) × L(mm) × 27.2
```

> 推导：PWM = 1500 + (φ × L / 15.625) × 2000/270 = 1500 + φ × L × (2000 / 4219) ≈ 1500 + φ × L / 2.11

#### 控制架构

```
K230球坐标 → error_x(cm) → 外环位置PID → φ_des(期望倾角)
    ├─ KP_ball × error              (比例)
    ├─ KD_ball × Δerror / Δt         (微分)
    ├─ Kff_accel × Δv_car / Δt       (车体加速度前馈)
    └─ Kff_curve × sgn(line_error)   (弯道离心力前馈)
         ↓
φ_des → PWM = 1500 + φ(deg) × L / 2.11  →  内环速率限幅  →  BSP_Servo_SetPWM
```

内环速率限幅（防急动）：每次限幅 3~5 PWM 步/10ms。

#### 待标定参数

| 参数 | 预估值 | 最终确定方式 |
|------|--------|------------|
| L (铰链到齿条距离) | ~200mm | 3D 打印组装后卡尺实测 |
| 静摩擦死区 φ_min | 0.1° ~ 0.3° | 静摩擦死区测试 |
| 系统增益 K | 待 φ_min 后计算 | 阶跃响应测试 + 后处理 |
| 球位置 KP | 待系统增益 K | 阶跃响应测试 |
| 球位置 KD | KP/10 ~ KP/5 | 阶跃响应测试 |
| 弯道前馈系数 Kff_curve | 理论算 + 调参 | v²/R |
| 加速度前馈系数 Kff_accel | 理论算 + 调参 | Δv/10ms |
| 舵机速度限幅 (MAX_RATE) | 3~5 PWM 步/10ms | 实测防急动 |

#### 标定流程（组装完成后）

1. **舵机 PWM 扫描**：PWM 500→2500μs，确认实际 0°/135°/270° 对应值
2. **齿轮齿条线性度**：PWM 500→2500μs 步长 100μs（20 点），每点测摆杆倾角，拟合直线，验证 R² > 0.99
3. **静摩擦死区**：球置中心 O，逐步增加倾角（~0.05°/step），记录球开始移动的最小倾角 φ_min
4. **阶跃响应**：φ = φ_min + 0.3°/0.8°/1.5°，各重复 3 次，BLE 记录 `<tick, ball_cm>`@100Hz，后处理获取系统增益 K 和纯延时


## 程序切换指南

> 当需要在不同 app 层循线模块之间切换时（如高速循线 ↔ 慢速直行循线），只需修改 `app/main.c` 中的 7 处引用，不需要改动其他任何文件。

### 当前可用的循线模块

| 模块 | 文件 | 函数前缀 | 用途 |
|------|------|----------|------|
| 高速循线 | `app/app_line_track_high_speed.c/h` | `APP_LineTrack_` | 0.35 m/s 高速 PD 循线，四通道全黑停车 + PI 主动刹车 |
| 慢速直行循线 | `app/app_line_track_low_speed_straight.c/h` | `APP_LineTrack_LowSpeedStraight_` | 匀加速起步 + 匀速直行，2m 自动停车，OLED 显示速度/距离 |
| 慢速环线循线 | `app/app_line_track_low_speed_circle.c/h` | `APP_LineTrack_LowSpeedCircle_` | 一圈匀加速 + 匀速环线，6.2m 自动停车，OLED 显示速度/距离 |

### 切换步骤

以"高速循线 → 慢速直行循线"为例：

**1. 修改 `#include`（1 处）**

```c
// #include "app_line_track_high_speed.h"       // 注释掉旧的
#include "app_line_track_low_speed_straight.h"   // 新的
```

**2. 修改 6 处函数调用**

在 `main.c` 中搜索旧模块的函数前缀（如 `APP_LineTrack_`），替换为新前缀（如 `APP_LineTrack_LowSpeedStraight_`）：

| 位置 | 旧调用（高速循线） | 新调用（慢速直行） |
|------|-------------------|-------------------|
| `on_timer_10ms()` | `APP_LineTrack_TimerTick()` | `APP_LineTrack_LowSpeedStraight_TimerTick()` |
| `on_key_click()` | `APP_LineTrack_IsRunning()` | `APP_LineTrack_LowSpeedStraight_IsRunning()` |
| `on_key_click()` | `APP_LineTrack_Stop()` | `APP_LineTrack_LowSpeedStraight_Stop()` |
| `on_key_click()` | `APP_LineTrack_Start()` | `APP_LineTrack_LowSpeedStraight_Start()` |
| `main()` 初始化 | `APP_LineTrack_Init()` | `APP_LineTrack_LowSpeedStraight_Init()` |
| `main()` 主循环 | `APP_LineTrack_Run()` | `APP_LineTrack_LowSpeedStraight_Run()` |

**3. 确认 `.c` 文件在 Keil 项目中**

两个模块的 `.c` 文件都应该已在 Keil 项目的 `app` Group 中。如果缺少目标文件，参考第 9 节"新文件加入 Keil 项目"。

**4. 重新编译**

Keil → Rebuild all targets。

### 注意事项

1. **互斥原则**：同一时间只能使用一个循线模块。两个模块的函数都会编译链接，但 `main.c` 只调用其中一个。若 `#include` 了两个头文件但只调用一个模块的函数，另一个模块的代码虽然链接了但不会被调用——浪费 Flash 空间，建议注释掉不用的 `#include`。

2. **命名规律**：所有 app 模块遵循统一的回调接口命名模式：
   - `<前缀>_Init()` — 初始化
   - `<前缀>_TimerTick()` — 10ms 定时器回调
   - `<前缀>_Run()` — 主循环调用
   - `<前缀>_Start()` / `_Stop()` / `_IsRunning()` — 按键控制

3. **app_control 不变**：`app_control.c`（速度 PI + BLE 指令）是独立模块，与循线模块无关，切换时不需要改动。

4. **MID_LineTrack 共用**：两个循线模块都依赖 `middleware/mid_line_track.c` 提供底层灰度 PD 计算，切换时 middleware 层完全不动。

5. **Keil Group 不区分方案**：`bsp_stepper.c` 和 `bsp_servo.c`（未来）可以同时存在于 BSP Group 中，由 `main.c` 的实际调用决定哪个参与运行。不需要从项目中移除文件，只需在 `main.c` 中切换 include 和函数调用。

6. **OLED 显示差异**：不同模块的 OLED 布局不同（高速循线显示速度曲线，慢速直行显示距离+目标速度），切换后留意显示是否与预期一致。
# DC Motor Car

**Version:** 1.4.0-dev

## 项目说明

基于 TI MSPM0G3507 的 **智能小车控制工程**，采用 **app-bsp-middleware-core 四层架构**。

当前 1.4-dev 分支已实现：
- **巡线算法** — 非线性 PD 巡线控制（查表法衰减 + 差速转向）
- **灰度传感器** — 8 路数字灰度传感器读取（多路复用 + GPIO 直接读取）
- **电机驱动** — TB6612 双路 H 桥 PWM 控制（正反转 + 占空比）
- **编码器测速** — 双路正交编码器 2x 解码（GPIO 中断 + ISR 内实时计数）
- **PI 速度闭环** — 增量式离散 PI 控制器（deadband + 低通滤波）
- **UART 通信** — 环形缓冲 + 中断收发
- **BLE 桥接** — JDY-16 透传模块，MCU 自定义指令集（!ENC 远程采样 + CSV 保存）
- **按键控制** — 单击检测状态机
- **10ms 定时器** — 周期性定时器 ISR（回调模式）
- **OLED 显示** — SSD1306 驱动（软件 SPI，含 6x12 / 8x16 ASCII 字库），显示左右轮目标/实际速度
- **LED 指示** — GPIO 控制（亮、灭、翻转、闪烁）
- **延时服务** — SysTick 精密延时（毫秒 / 微秒）
- **四层架构** — app / bsp / middleware / core 分层落地，包含完整编码规范
- **JY62 陀螺仪驱动** — 六轴姿态数据 UART 状态机解析（角速度/角度/加速度）
- **DMA 循环接收** — UART2 + DMA 乒乓缓冲，零中断逐字回调
- **陀螺仪航向锁** — 比例纠偏控制器（deadband + 限幅）
- **陀螺仪任务调度** — 原地转向 + 距离驱动状态机
- **K230 视觉识别** — UART_1 直连 K230，解析 YOLOv5 检测结果，1/2 数字识别 + 路口转弯
- **按键启动** — 单击按键启动当前 App 模式（LINE/VISION），单击停止

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
| UART_1 | PB6 | PB7 | 115200 | K230 视觉模块 |
| UART_2 | PA21 | PB16 | 115200 (DMA) | JY62 陀螺仪 |

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

## 按键启动

**硬件按键**: PA18（封装引脚 11）

**操作逻辑**（`main.c`）:

```
运行中按 → 紧急停止（所有 App 停止，电机停转）
空闲时按 → 启动当前 App 模式：
  ├─ APP_MODE_VISION  → 视觉识别任务
  │   K230 识别数字 → 循迹 → 过路口 → 转弯 → 二次循迹 → 停止
  └─ APP_MODE_LINE_TRACK → 循迹任务
      按键触发后开始循迹
```

**切换默认模式**：通过串口发送 `!MODE LINE` 或 `!MODE VISION`，或修改 `app_control.c` 的初始值。

**安全联锁**：必须先按键 Arm 一次，电机才能转动（`s_armed` 标志位）。

## 目录结构

```
├── app/                 # 应用层
│   ├── main.c           # 入口：初始化 + 调度
│   ├── app_control.c    # 应用控制：PI 速度闭环 + 显示更新
│   ├── app_control.h
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
| 轮子每圈脉冲数 | **25525** pulses/rev（1m 行驶实测校正） |
| 每脉冲距离 | 0.00800 mm/pulse |
| 理论值 | 500 × 2 × 28 = 28000（与实际偏差 ~8.8%） |

> 校准方法：上电后在 OLED 上观察 Enc A/B 累计脉冲，手动转动轮子恰好一圈，读取脉冲差值。
> 该值已用于 `app_gyro_task.c` 的距离计算。

### 速度闭环 PI (app_control.c)

| 参数 | 数值 | 说明 |
|------|------|------|
| KP | 600 | 增量式离散 PI |
| KI | 450 | |

```
pwm += KP × (bias - last_bias) + KI × bias
```

### 灰度循线 PD (mid_line_track.c)

**全局参数：**

| 参数 | 数值 | 说明 |
|------|------|------|
| `MID_TRACK_BASE_SPEED` | 0.50 m/s | 直行速度 |
| `MID_TRACK_LEFT_BIAS` | 0.02 | 左转死区补偿 |
| `MID_TRACK_SEARCH_SPEED` | 0.28 m/s | 丢线搜索速度 |
| `MID_TRACK_SEARCH_KP` | 0.038 | 丢线搜索比例增益 |
| `MID_TRACK_SEARCH_CAP` | 6 | 丢线误差限幅 |

**传感器权重 (s_track_weights):**

```
[-10, -6, -3, -1, 1, 3, 6, 10]
```

**PD 查表 (按 |error| 索引)：**

| \|error\| | attenuation | KP | KD | 转弯半径 R |
|-----------|------------|------|------|-----------|
| 0 | 1.00 | 0.0058 | 0.0040 | ∞ (直行) |
| 1 | 0.95 | 0.0090 | 0.006  | ~5.5m |
| 2 | 0.82 | 0.0144 | 0.009  | ~1.5m |
| 3 | 0.62 | 0.0140 | 0.009  | ~0.78m |
| 4 | 0.47 | 0.0114 | 0.007  | ~0.54m |
| 5 | 0.37 | 0.0090 | 0.006  | ~0.43m |
| 6 | 0.28 | 0.0148 | 0.0095 | ~0.17m |
| 7 | 0.22 | 0.0132 | 0.009  | ~0.13m |

> 算法：`error = Σ(w[i] × sensor[i]) / Σ(sensor[i])`
>
> `base_speed = BASE_SPEED × attenuation[|error|]`
>
> `correction = KP[|error|] × error + KD[|error|] × d(error)/dt`
>
> `left = base_speed + correction`, `right = base_speed - correction`
>
> 转弯半径：`R = base_speed × 0.21 / (2 × KP × |error|)`（忽略 KD 项稳态近似）

**丢线行为：**
- 全白 (total_intensity < 1) → 向 `s_last_valid_error` 方向搜索
- 全黑 (8 路全触发) → 直行

## 后续计划

### 2.0 上半 — 控制优化与调试工具链

- **引入 KD** — 速度闭环从 PI 升级为 PID，微分项抑制超调，提高动态响应
- **蓝牙指令集扩展**：
  - `!SPD <left> <right>` — 手动设置左右轮目标速度（m/s）
  - `!PID <kp> <ki> <kd>` — 在线调整速度闭环 PID 权重，实时生效
  - `!SWP <start> <end> <step> <duration_ms>` — 自动扫速计划：按步长递增目标速度，定时记录实际速度，保存 CSV
- **PC 工具同步** — `tools/ble/jdy16_ble.py` 增加 `!SWP` 触发、CSV 保存、实时绘图辅助调参
- **灰度循迹优化** — 重新设计 error 映射表 + PD 参数整定，适应更复杂赛道

### 2.0 下半 — 扩展外设

- **二维数字云台控制** — PWM 舵机驱动，角度控制接口
- **K230 视觉模块** — 串口通信协议封装，视觉数据处理与融合
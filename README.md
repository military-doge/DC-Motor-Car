# DC Motor Car

**Version:** 1.0.0-dev

## 项目说明

基于 TI MSPM0G3507 的 **智能小车控制工程**，采用 **app-bsp-middleware-core 四层架构**。

当前 1.0-dev 分支已实现：
- **OLED 显示** — SSD1306 驱动（软件 SPI，含 6x12 / 8x16 ASCII 字库）
- **LED 指示** — GPIO 控制（亮、灭、翻转、闪烁）
- **延时服务** — SysTick 精密延时（毫秒 / 微秒）
- **四层架构** — app / bsp / middleware / core 分层落地，包含完整编码规范

## 目录结构

```
├── app/                 # 应用层
│   └── main.c           # 入口：初始化 + 调度
├── bsp/                 # 板级支持包
│   ├── bsp_delay.c/h    # SysTick 延时（ms / us）
│   └── bsp_led.c/h      # GPIO LED 控制
├── middleware/          # 中间件层
│   ├── mid_oled.c/h     # SSD1306 OLED 驱动（framebuffer）
│   └── mid_oledfont.h   # ASCII 字库（6x12 / 8x16）
├── core/                # 核心层
│   ├── ti_msp_dl_config.c/h  # SysConfig 生成代码
│   ├── startup_mspm0g350x_uvision.s
│   └── DC-Motor-Car.syscfg   # 外设配置源文件
├── tools/keil/          # SysConfig 工具链集成
├── sdk_config.ini       # Windows TI SDK / SysConfig 路径配置
├── apply_sdk_paths.bat  # 根据 sdk_config.ini 更新 .uvprojx 路径
└── DC-Motor-Car.uvprojx  # Keil 项目文件
```

## 分层架构与开发规范

本项目采用 **app / bsp / middleware / core** 四层架构，各层职责和依赖关系如下：

| 层 | 职责 | 可包含的依赖 | 当前模块 |
|----|------|-------------|---------|
| **app/** | 应用逻辑：主循环、状态机、控制算法 | middleware/、bsp/、core/ | `main.c`（入口） |
| **middleware/** | 协议/融合层：传感器数据处理、通信协议、算法抽象 | bsp/、core/ | `mid_oled`（SSD1306 驱动） |
| **bsp/** | 板级驱动：外设封装（GPIO、UART、PWM、ADC 等） | core/（`ti_msp_dl_config.h`）及标准库 | `bsp_delay`（延时）、`bsp_led`（LED） |
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
    BSP_LED_Init();
    BSP_UART_Init();
    BSP_Delay_Init();       // 延迟模块（如使用定时器中断模式）
    // ... 其他 BSP 模块

    // [3] Middleware 层初始化
    MID_OLED_Init();
    // ... 其他 Middleware 模块

    // [4] App 层初始化
    APP_Control_Init();
    // ... 其他 App 模块

    while (1) {
        APP_Control_Run();  // 主循环调度
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
- [ ] 新 .c 文件已加入 Keil 项目对应 Group？
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

## 后续计划

### 1.1 dev — 电机控制与传感器

当前 1.0-dev 已完成基础显示和指示功能，后续计划：
- **电机驱动** — TB6612 / 直流电机 PWM 控制
- **编码器** — 正交编码器测速
- **PID 控制** — 速度闭环

> 新功能开发前先修改 `.syscfg` 添加外设，再编写对应 `bsp_` / `mid_` / `app_` 模块。
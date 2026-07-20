# BLE 蓝牙工具

PC 端蓝牙调试工具，通过 JDY-16 BLE 透传模块与 MCU 通信。

## 环境依赖

```bash
pip install bleak
```

Windows 用户需确保蓝牙已开启。

## jdy16_ble.py — BLE 桥接命令行工具

执行 MCU 自定义指令，用于小车运行时远程调试和数据采集。

```bash
python jdy16_ble.py
```

**原理:**
```
PC ←→ BLE ←→ JDY-16 ←→ UART ←→ MCU(mid_ble.c)
                    ↑
             MCU 自定义 ! 指令
```

---

## MCU 指令集

### !ENC — 电机转速定时采样

定时读取左右电机转速 (m/s)，支持实时输出和 CSV 保存。

**格式:** `!ENC <持续时长> <采样间隔> [save]`

| 参数 | 说明 | 单位 | 范围 |
|------|------|------|------|
| `dur` | 持续采样时长 | ms | 1 ~ 30000 |
| `int` | 采样间隔 | ms | 10 ~ 30000 |
| `save` | 可选，加 `save`/`s`/`1` 保存 CSV | - | - |

**示例:**
```
> !ENC 5000 100            # 每100ms读一次，持续5秒，实时输出
发送: !ENC 5000 100
← OK ENC:5000ms int:100ms
← ENC:0.100,0.098
← ENC:0.102,0.099
← ...
← ENC_DONE

> !ENC 3000 50 save         # 每50ms读一次，持续3秒，保存到CSV
发送: !ENC 3000 50 save
← OK ENC:3000ms int:50ms [SAVE]
← ENC_DONE
[√] 编码器数据已保存: ...\log\enc_20260719_143025.csv
    采样点数: 60, 间隔: 50ms
```

CSV 文件格式:
```csv
time_ms,speed_a_mps,speed_b_mps
0,0.100000,0.098000
50,0.102000,0.099000
100,0.101000,0.097000
...
```

---

## 内置命令

| 命令 | 说明 |
|------|------|
| `.scan` | 重新扫描 BLE 设备 |
| `.info` | 显示当前连接信息和 ENC 录制状态 |
| `.disconnect` | 断开连接 |
| `.reconnect` | 重新连接上次设备 |
| `.help` | 显示帮助 |
| `.quit` | 退出程序 |

---

## 目录结构

```
tools/ble/
├── README.md              # 本文件
├── jdy16_ble.py           # BLE 桥接工具
└── log/                   # ENC CSV 输出目录
    └── enc_*.csv
```

# YOLOv5n 逐帧推理 → LCD 显示
# 适用于 CanMV K230 + 3.5寸 MIPI LCD (ST7701)

import time
import uos

from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, Pin, UART

import nncase_runtime as nn
import ulab.numpy as np
import image
import gc

# ===================== 可配置参数 =====================

# Sensor 输出分辨率（全画幅 480×288）
CAM_W = 480
CAM_H = 288

# 取中央 64 行送入模型
CAM_STRIP_H = 64
CROP_Y = (CAM_H - CAM_STRIP_H) // 2  # 自动计算中央起始行

# 模型输入分辨率
MODEL_W = 480
MODEL_H = 64

# LCD 物理分辨率（ST7701 原生 800×480，剩余区域显示日志）
DISP_W = 800
DISP_H = 480

# 摄像头画面在 LCD 上的位置（左上角）
CAM_X = 0
CAM_Y = 0

# 模型权重路径
KMODEL_PATH = "/data/model/model.kmodel"

# 后处理参数
CONF_THRESH = 0.68
NMS_IOU_THRESH = 0.45

# 检测框颜色 (BGR) — 统一黄色, 与采集脚本标记风格一致
DET_COLOR = (0, 255, 255)  # 黄色

# 串口参数 (UART2: TX=11, RX=12)
UART_ID = 2
UART_BAUD = 115200
UART_TX_PIN = 11
UART_RX_PIN = 12

# 分度线参数（在 480×64 裁切区域内的 x 坐标）
COLOR = (0, 255, 0)  # 绿色 (BGR) — 所有刻度线共用

# 主分度线
MAIN_HEIGHT = 64        # 主刻度线高度（贯穿裁切区域）
ZERO_X = 240            # 零点分度线

# 次级分度线
SUB_HEIGHT = 48           # 次级刻度线高度
SUB_OFFSET_50_LEFT = 84   # -50mm 偏移（像素）
SUB_OFFSET_50_RIGHT = 84  # +50mm 偏移（像素）
SUB_OFFSET_100_LEFT = 170 # -100mm 偏移（像素）
SUB_OFFSET_100_RIGHT = 165# +100mm 偏移（像素）

# 其余分度线（0–120mm 整 10mm，跳过已有 50/100mm）
FILL_HEIGHT = 32          # 刻度线高度
OFFSET_10_LEFT = 17       # -10mm 偏移（像素）
OFFSET_10_RIGHT = 17      # +10mm 偏移（像素）
OFFSET_20_LEFT = 34       # -20mm
OFFSET_20_RIGHT = 34      # +20mm
OFFSET_30_LEFT = 50       # -30mm
OFFSET_30_RIGHT = 50      # +30mm
OFFSET_40_LEFT = 67       # -40mm
OFFSET_40_RIGHT = 67      # +40mm
OFFSET_60_LEFT = 101      # -60mm
OFFSET_60_RIGHT = 101     # +60mm
OFFSET_70_LEFT = 118      # -70mm
OFFSET_70_RIGHT = 118     # +70mm
OFFSET_80_LEFT = 134      # -80mm
OFFSET_80_RIGHT = 134     # +80mm
OFFSET_90_LEFT = 151      # -90mm
OFFSET_90_RIGHT = 151     # +90mm
OFFSET_110_LEFT = 185     # -110mm
OFFSET_110_RIGHT = 185    # +110mm
OFFSET_120_LEFT = 202     # -120mm
OFFSET_120_RIGHT = 202    # +120mm

# 末端分度线
END_HEIGHT = 64           # 末端刻度线高度
END_OFFSET_LEFT = 212     # -125mm 偏移（像素）
END_OFFSET_RIGHT = 205    # +125mm 偏移（像素）


# =====================================================

# 日志缓冲区
_LOG = []
_MAX_LOG = 40


def log(msg):
    """添加到 LCD 日志面板"""
    _LOG.append(str(msg))
    if len(_LOG) > _MAX_LOG:
        _LOG.pop(0)
    print(msg)


# ========== 初始化 KEY 按键 (GPIO21) ==========
fpioa = FPIOA()
fpioa.set_function(21, FPIOA.GPIO21)
KEY = Pin(21, Pin.IN, Pin.PULL_UP)
_key_prev = 1  # 上次按键状态 (1=松开)

# ========== 初始化串口 (UART2) 发送小球位置 ==========
fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
uart = UART(UART_ID, UART_BAUD, bits=8, parity=0, stop=1, timeout=1000)


# ========== 1. 初始化摄像头 ==========
sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=CAM_W, height=CAM_H)
sensor.set_pixformat(Sensor.RGB565)

# ========== 2. 初始化 LCD 显示屏 ==========
Display.init(Display.ST7701, to_ide=True)

# ========== 3. 创建显示缓冲区（800×480） ==========
display_buf = image.Image(DISP_W, DISP_H, image.RGB565)

# ========== 4. 启动媒体引擎和摄像头 ==========
MediaManager.init()
sensor.run()

# ========== 5. 加载 AI 模型 ==========
log("[AI] checking model file...")
try:
    uos.stat(KMODEL_PATH)
    log("[AI] model file exists")
except:
    log("[AI] model file NOT FOUND at " + KMODEL_PATH)
    raise SystemExit(1)

kpu = nn.kpu()
log("[AI] loading model...")
kpu.load_kmodel(KMODEL_PATH)
hw = MODEL_H * MODEL_W
INPUT_BUF = bytearray(hw * 3)

# 干跑（warm-up）
dummy_np = np.zeros((1, 3, MODEL_H, MODEL_W), dtype=np.uint8)
t = nn.from_numpy(dummy_np)
kpu.set_input_tensor(0, t)
kpu.run()
kpu.get_output_tensor(0)
log("[AI] model ready")

# ========== 6. 后处理函数 ==========
CLASS_NAMES = ["steel_ball"]


def _iou(ax, ay, aw, ah, bx, by, bw, bh):
    a_x1 = ax - aw / 2
    a_y1 = ay - ah / 2
    a_x2 = ax + aw / 2
    a_y2 = ay + ah / 2
    b_x1 = bx - bw / 2
    b_y1 = by - bh / 2
    b_x2 = bx + bw / 2
    b_y2 = by + bh / 2
    x1 = max(a_x1, b_x1)
    y1 = max(a_y1, b_y1)
    x2 = min(a_x2, b_x2)
    y2 = min(a_y2, b_y2)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = aw * ah
    area_b = bw * bh
    return inter / (area_a + area_b - inter + 1e-6)


def parse_detections(out_np, conf_th, nms_th):
    """
    YOLOv5 输出 → NMS → 按 x 排序
    返回 [(cls_id, conf, cx, cy, w, h), ...]
    """
    data = out_np
    while len(data.shape) > 2:
        data = data[0]
    ncols = data.shape[1]
    n_cls = ncols - 5  # 类别数

    raw = []
    for i in range(data.shape[0]):
        obj = float(data[i, 4])
        if obj < 0.2:
            continue
        for j in range(n_cls):
            c = obj * float(data[i, 5 + j])
            if c > conf_th:
                raw.append((c, float(data[i, 0]), float(data[i, 1]),
                            float(data[i, 2]), float(data[i, 3]), j))

    if not raw:
        return []

    # NMS
    raw = sorted(raw, key=lambda d: d[0], reverse=True)
    keep = []
    while raw:
        best = raw.pop(0)
        keep.append(best)
        remaining = []
        _, cx, cy, w, h = best[:5]
        for d in raw:
            bx, by, bw, bh = d[1:5]
            if _iou(cx, cy, w, h, bx, by, bw, bh) < nms_th:
                remaining.append(d)
        raw = remaining

    # 每类只保留最高置信度 (部署环境固定 1 个 ball)
    best_per_class = {}
    for d in keep:
        cls_id = d[5]
        conf = d[0]
        if cls_id not in best_per_class or conf > best_per_class[cls_id][0]:
            best_per_class[cls_id] = d
    keep = list(best_per_class.values())

    # 按 x 坐标从左到右排序
    keep.sort(key=lambda d: d[1])
    return [(d[5], d[0], d[1], d[2], d[3], d[4]) for d in keep]


# ========== 6.5 位置推算（基于标定刻度） ==========
# 标定表: (相对零点的像素偏移, 毫米位置), 由分度线参数自动生成
CALIB = [
    (-END_OFFSET_LEFT, -125),
    (-OFFSET_120_LEFT, -120),
    (-OFFSET_110_LEFT, -110),
    (-SUB_OFFSET_100_LEFT, -100),
    (-OFFSET_90_LEFT, -90),
    (-OFFSET_80_LEFT, -80),
    (-OFFSET_70_LEFT, -70),
    (-OFFSET_60_LEFT, -60),
    (-SUB_OFFSET_50_LEFT, -50),
    (-OFFSET_40_LEFT, -40),
    (-OFFSET_30_LEFT, -30),
    (-OFFSET_20_LEFT, -20),
    (-OFFSET_10_LEFT, -10),
    (0, 0),
    (OFFSET_10_RIGHT, 10),
    (OFFSET_20_RIGHT, 20),
    (OFFSET_30_RIGHT, 30),
    (OFFSET_40_RIGHT, 40),
    (SUB_OFFSET_50_RIGHT, 50),
    (OFFSET_60_RIGHT, 60),
    (OFFSET_70_RIGHT, 70),
    (OFFSET_80_RIGHT, 80),
    (OFFSET_90_RIGHT, 90),
    (SUB_OFFSET_100_RIGHT, 100),
    (OFFSET_110_RIGHT, 110),
    (OFFSET_120_RIGHT, 120),
    (END_OFFSET_RIGHT, 125),
]


def cx_to_mm(cx):
    """钢球检测框中心 x 像素 → 位置 mm（分段线性插值）"""
    delta = cx - ZERO_X
    if delta <= CALIB[0][0]:
        return CALIB[0][1]
    if delta >= CALIB[-1][0]:
        return CALIB[-1][1]
    for i in range(len(CALIB) - 1):
        p0, m0 = CALIB[i]
        p1, m1 = CALIB[i + 1]
        if p0 <= delta <= p1:
            t = (delta - p0) / (p1 - p0)
            return m0 + t * (m1 - m0)
    return 0.0


# ========== 7. 主循环 ==========
clock = time.clock()
last_fps = 0
fps_frame_cnt = 0
frame_count = 0

try:
    while True:
        clock.tick()
        fps_frame_cnt += 1
        frame_count += 1

        # --- 检测 KEY 按键: 打印当前帧信息 ---
        key_val = KEY.value()
        if key_val == 0 and _key_prev == 1:
            log("[KEY] pressed")
        _key_prev = key_val

        img = sensor.snapshot()

        # --- 裁剪中央 64 行送模型推理 ---
        strip = img.copy(roi=(0, CROP_Y, MODEL_W, CAM_STRIP_H))
        rgb = strip.to_rgb888()
        rgb_np = rgb.to_numpy_ref()  # (MODEL_H, MODEL_W, 3), HWC uint8
        rgb_2d = rgb_np.reshape((hw, 3))
        INPUT_BUF[:hw] = rgb_2d[:, 0].flatten().tobytes()    # R plane
        INPUT_BUF[hw:hw * 2] = rgb_2d[:, 1].flatten().tobytes()  # G plane
        INPUT_BUF[hw * 2:hw * 3] = rgb_2d[:, 2].flatten().tobytes()  # B plane
        arr = np.frombuffer(INPUT_BUF, dtype=np.uint8).reshape((1, 3, MODEL_H, MODEL_W))

        # --- KPU 推理 ---
        kpu.set_input_tensor(0, nn.from_numpy(arr))
        kpu.run()
        out_all = kpu.get_output_tensor(0).to_numpy()

        # --- 后处理 (坐标在 480×64 空间) ---
        dets = parse_detections(out_all, CONF_THRESH, NMS_IOU_THRESH)

        # 映射到全画幅 (480×288) 坐标空间
        dets_cam = []
        for d in dets:
            cls_id, c, cx, cy, w, h = d
            dets_cam.append((cls_id, c, cx, cy + CROP_Y, w, h))
        dets = dets_cam

        # ===== 立即推算小球位置并串口发送 (不等绘图/显示) =====
        tick = time.ticks_ms()
        if dets:
            ball_mm = cx_to_mm(dets[0][2])
            uart.write("X:%+.1f,T:%d,D:1\r\n" % (ball_mm, tick))
        else:
            uart.write("X:--,T:%d,D:0\r\n" % tick)

        # 画裁切区域标记 (黄色框, 同采集脚本风格)
        img.draw_rectangle(0, CROP_Y, CAM_W, CAM_STRIP_H,
            color=(0, 255, 255), thickness=2)

        # 画零点分度线（贯穿裁切区域）
        img.draw_line(ZERO_X, CROP_Y, ZERO_X, CROP_Y + MAIN_HEIGHT,
            color=COLOR, thickness=2)

        # 画 ±50mm 刻度线
        sub_y0 = CROP_Y + (CAM_STRIP_H - SUB_HEIGHT) // 2
        sub_y1 = sub_y0 + SUB_HEIGHT
        img.draw_line(ZERO_X - SUB_OFFSET_50_LEFT, sub_y0, ZERO_X - SUB_OFFSET_50_LEFT, sub_y1,
            color=COLOR, thickness=1)
        img.draw_line(ZERO_X + SUB_OFFSET_50_RIGHT, sub_y0, ZERO_X + SUB_OFFSET_50_RIGHT, sub_y1,
            color=COLOR, thickness=1)

        # 画 ±100mm 刻度线
        img.draw_line(ZERO_X - SUB_OFFSET_100_LEFT, sub_y0, ZERO_X - SUB_OFFSET_100_LEFT, sub_y1,
            color=COLOR, thickness=1)
        img.draw_line(ZERO_X + SUB_OFFSET_100_RIGHT, sub_y0, ZERO_X + SUB_OFFSET_100_RIGHT, sub_y1,
            color=COLOR, thickness=1)

        # 画 ±125mm 末端刻度线
        end_y0 = CROP_Y + (CAM_STRIP_H - END_HEIGHT) // 2
        end_y1 = end_y0 + END_HEIGHT
        img.draw_line(ZERO_X - END_OFFSET_LEFT, end_y0, ZERO_X - END_OFFSET_LEFT, end_y1,
            color=COLOR, thickness=1)
        img.draw_line(ZERO_X + END_OFFSET_RIGHT, end_y0, ZERO_X + END_OFFSET_RIGHT, end_y1,
            color=COLOR, thickness=1)

        # 画 0–120mm 整 10mm 刻度线（跳过已有 50/100mm）
        fill_y0 = CROP_Y + (CAM_STRIP_H - FILL_HEIGHT) // 2
        fill_y1 = fill_y0 + FILL_HEIGHT
        img.draw_line(ZERO_X - OFFSET_10_LEFT, fill_y0, ZERO_X - OFFSET_10_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_10_RIGHT, fill_y0, ZERO_X + OFFSET_10_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_20_LEFT, fill_y0, ZERO_X - OFFSET_20_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_20_RIGHT, fill_y0, ZERO_X + OFFSET_20_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_30_LEFT, fill_y0, ZERO_X - OFFSET_30_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_30_RIGHT, fill_y0, ZERO_X + OFFSET_30_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_40_LEFT, fill_y0, ZERO_X - OFFSET_40_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_40_RIGHT, fill_y0, ZERO_X + OFFSET_40_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_60_LEFT, fill_y0, ZERO_X - OFFSET_60_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_60_RIGHT, fill_y0, ZERO_X + OFFSET_60_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_70_LEFT, fill_y0, ZERO_X - OFFSET_70_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_70_RIGHT, fill_y0, ZERO_X + OFFSET_70_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_80_LEFT, fill_y0, ZERO_X - OFFSET_80_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_80_RIGHT, fill_y0, ZERO_X + OFFSET_80_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_90_LEFT, fill_y0, ZERO_X - OFFSET_90_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_90_RIGHT, fill_y0, ZERO_X + OFFSET_90_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_110_LEFT, fill_y0, ZERO_X - OFFSET_110_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_110_RIGHT, fill_y0, ZERO_X + OFFSET_110_RIGHT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X - OFFSET_120_LEFT, fill_y0, ZERO_X - OFFSET_120_LEFT, fill_y1, color=COLOR, thickness=1)
        img.draw_line(ZERO_X + OFFSET_120_RIGHT, fill_y0, ZERO_X + OFFSET_120_RIGHT, fill_y1, color=COLOR, thickness=1)

        # 画检测框 + 置信度
        for d in dets:
            cls_id, c, cx, cy, w, h = d
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            bw = int(w)
            bh = int(h)
            img.draw_rectangle(x1, y1, bw, bh, color=DET_COLOR, thickness=2)
            label = "%s %.2f" % (CLASS_NAMES[cls_id], c)
            img.draw_string_advanced(x1, y1 - 18, 18, label, color=DET_COLOR)
            # 中心十字准星 (ball 6px)
            cxi, cyi = int(cx), int(cy)
            arm = 6  # ball 十字臂长
            img.draw_line(cxi - arm, cyi, cxi + arm, cyi, color=DET_COLOR, thickness=1)
            img.draw_line(cxi, cyi - arm, cxi, cyi + arm, color=DET_COLOR, thickness=1)

        # ========== 合成 LCD 显示画面 ==========
        display_buf.draw_rectangle(0, 0, DISP_W, DISP_H, color=(0, 0, 0), fill=True)

        # 粘贴摄像头画面到左上角
        display_buf.draw_image(img, CAM_X, CAM_Y)

        # --- 摄像头下方：状态信息 ---
        info_y = CAM_Y + CAM_H + 10
        count = len(dets)
        display_buf.draw_string_advanced(10, info_y, 28,
            str(count) + " found" if count else "none", color=(255, 255, 0))
        if fps_frame_cnt % 30 == 0:
            last_fps = int(clock.fps())
        display_buf.draw_string_advanced(10, info_y + 32, 28,
            str(last_fps) + " fps", color=(255, 255, 0))

        # 钢球位置显示 (mm) — 已在上方推理后提前串口发送
        pos_str = "pos: %+.1f mm" % ball_mm if dets else "pos: --"
        display_buf.draw_string_advanced(10, info_y + 64, 28,
            pos_str, color=(0, 255, 0))

        # --- 右侧日志面板 ---
        tx = CAM_W + 15
        ty = 10
        for line in _LOG:
            display_buf.draw_string_advanced(tx, ty, 18,
                line, color=(100, 255, 100))
            ty += 18
            if ty > DISP_H - 10:
                break

        Display.show_image(display_buf)


        gc.collect()

except KeyboardInterrupt:
    log("[INFO] user interrupted")
finally:
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    log("[INFO] resources released")

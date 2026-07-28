/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * mid_ble.c — BLE 桥接中间件 (JDY-16)
 *
 * 处理 MCU 自定义指令（! 前缀）。
 *
 * 指令集:
 *   !SPD <dur> <int> [save] → 定时采样电机转速 (m/s)
 *
 * 依赖: bsp_uart (PB6=TX, PB7=RX 连接 JDY-16)
 */

#include "mid_ble.h"
#include "bsp_uart.h"
#include "bsp_delay.h"
#include "app_control.h"
#include <stdlib.h>
#include <string.h>

/* ---- 常量 ---- */

#define LINE_BUF_SIZE   128
#define SPD_BUF_MAX     512
#define SPD_DUMP_BATCH  8

/* ---- 编码器采样任务 ---- */

typedef struct {
    bool     active;
    bool     dumping;       /* true = 正在分批输出 save 数据 */
    bool     save;          /* true = 缓冲输出, false = 实时输出 */
    uint16_t interval_ms;   /* 采样间隔 (ms) */
    uint16_t duration_ms;   /* 总持续时长 (ms) */
    uint16_t buf_count;     /* 实际采样数 */
    uint16_t dump_index;    /* dump 进度 */
    float    buf_a[SPD_BUF_MAX];
    float    buf_b[SPD_BUF_MAX];
} mid_ble_spd_task_t;

/* ---- 静态状态 ---- */

static uint8_t  s_line_buf[LINE_BUF_SIZE];
static uint16_t s_line_len;

static mid_ble_spd_task_t s_spd_task;
static uint32_t s_spd_start_ms;
static uint32_t s_spd_last_ms;

/* ---- 辅助函数 ---- */

static void ble_uart_send_int(int32_t val)
{
    char buf[12];
    int i = 0;
    bool neg = false;
    if (val < 0) { neg = true; val = -val; }
    do {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0 && i < 11);
    if (neg) buf[i++] = '-';
    while (i > 0) BSP_UART_SendByte((uint8_t)buf[--i]);
}

static void ble_uart_send_float(float val, uint8_t decimals)
{
    if (val < 0.0f) { BSP_UART_SendByte('-'); val = -val; }
    uint32_t int_part = (uint32_t)val;
    ble_uart_send_int((int32_t)int_part);
    BSP_UART_SendByte('.');
    float frac = val - (float)int_part;
    uint8_t i;
    for (i = 0; i < decimals; i++) {
        frac *= 10.0f;
        uint8_t d = (uint8_t)frac;
        BSP_UART_SendByte('0' + d);
        frac -= (float)d;
    }
}

/* ---- 命令处理 ---- */

/* !SPD <duration_ms> <interval_ms> [save]
 *   duration_ms: 持续时长 (ms), 最短 = interval_ms
 *   interval_ms: 采样间隔 (ms), 最小 10ms
 *   save: 可选 "1" 或 "save" → 结束时一次性发送全部数据 */
static void ble_handle_spd_command(const char *cmd)
{
    int dur_ms = 0, int_ms = 100, save = 0;

    const char *p = cmd + 4;
    while (*p == ' ') p++;

    dur_ms = atoi(p);
    while (*p >= '0' && *p <= '9') p++;
    while (*p == ' ') p++;
    int_ms = atoi(p);
    while (*p >= '0' && *p <= '9') p++;
    while (*p == ' ') p++;
    if (*p == '1' || *p == 's' || *p == 'S') save = 1;

    if (int_ms < 10) int_ms = 10;
    if (dur_ms <= 0) dur_ms = int_ms;
    if (dur_ms > 30000) dur_ms = 30000;

    {
        uint16_t max_samples = (uint16_t)(dur_ms / int_ms);
        if (max_samples > SPD_BUF_MAX) {
            dur_ms = (int)(int_ms * SPD_BUF_MAX);
        }
    }

    s_spd_task.active      = true;
    s_spd_task.dumping     = false;
    s_spd_task.interval_ms = (uint16_t)int_ms;
    s_spd_task.duration_ms = (uint16_t)dur_ms;
    s_spd_task.save        = (bool)save;
    s_spd_task.buf_count   = 0;
    s_spd_task.dump_index  = 0;
    s_spd_start_ms         = BSP_Delay_GetTick();
    s_spd_last_ms          = s_spd_start_ms;

    BSP_UART_SendString("OK SPD:");
    ble_uart_send_int(dur_ms);
    BSP_UART_SendString("ms int:");
    ble_uart_send_int(int_ms);
    BSP_UART_SendString("ms");
    if (save) BSP_UART_SendString(" [SAVE]");
    BSP_UART_SendString("\r\n");
}

/* ---- 编码器任务轮询 ---- */

static void ble_spd_task_poll(void)
{
    uint16_t i;

    if (!s_spd_task.active) return;

    if (s_spd_task.dumping) {
        uint16_t start = s_spd_task.dump_index;
        uint16_t end   = start + SPD_DUMP_BATCH;
        if (end > s_spd_task.buf_count) end = s_spd_task.buf_count;

        for (i = start; i < end; i++) {
            BSP_UART_SendString("SPD:");
            ble_uart_send_float(s_spd_task.buf_a[i], 3);
            BSP_UART_SendByte(',');
            ble_uart_send_float(s_spd_task.buf_b[i], 3);
            BSP_UART_SendString("\r\n");
        }
        s_spd_task.dump_index = end;

        if (end >= s_spd_task.buf_count) {
            BSP_UART_SendString("SPD_DONE\r\n");
            s_spd_task.active  = false;
            s_spd_task.dumping = false;
        }
        return;
    }

    {
        uint32_t now     = BSP_Delay_GetTick();
        uint32_t elapsed = now - s_spd_start_ms;

        if ((now - s_spd_last_ms) >= s_spd_task.interval_ms) {
            s_spd_last_ms = now;

            float speed_a = APP_Control_GetSpeedA();
            float speed_b = APP_Control_GetSpeedB();

            if (s_spd_task.save) {
                if (s_spd_task.buf_count < SPD_BUF_MAX) {
                    s_spd_task.buf_a[s_spd_task.buf_count] = speed_a;
                    s_spd_task.buf_b[s_spd_task.buf_count] = speed_b;
                    s_spd_task.buf_count++;
                }
            } else {
                BSP_UART_SendString("SPD:");
                ble_uart_send_float(speed_a, 3);
                BSP_UART_SendByte(',');
                ble_uart_send_float(speed_b, 3);
                BSP_UART_SendString("\r\n");
            }
        }

        if (elapsed >= s_spd_task.duration_ms) {
            if (s_spd_task.save && s_spd_task.buf_count > 0) {
                s_spd_task.dumping    = true;
                s_spd_task.dump_index = 0;
            } else {
                BSP_UART_SendString("SPD_DONE\r\n");
                s_spd_task.active = false;
            }
        }
    }
}

/* !DRIVE <speed> — 设置速度直行 (BLE direct drive) */
static void ble_handle_drive_command(const char *cmd)
{
    float speed = 0.10f;
    const char *p = cmd + 6;
    while (*p == ' ') p++;
    if (*p >= '0' && *p <= '9') {
        speed = (float)atof(p);
    }
    if (speed < 0.05f)  speed = 0.05f;
    if (speed > 0.80f)  speed = 0.80f;

    APP_Control_StartDirect(speed);
    BSP_UART_SendString("OK DRIVE speed:");
    ble_uart_send_float(speed, 2);
    BSP_UART_SendString("m/s\r\n");
}

/* !SWP <kp> <ki> — 速度扫频测试 */
static void ble_handle_swp_command(const char *cmd)
{
    float kp = 1225.0f, ki = 3600.0f;
    const char *p = cmd + 4;
    while (*p == ' ') p++;
    kp = (float)atof(p);
    while ((*p >= '0' && *p <= '9') || *p == '.' || *p == '-') p++;
    while (*p == ' ') p++;
    ki = (float)atof(p);

    APP_Control_StartSweep(kp, ki);
    BSP_UART_SendString("OK SWP kp:");
    ble_uart_send_float(kp, 1);
    BSP_UART_SendString(" ki:");
    ble_uart_send_float(ki, 1);
    BSP_UART_SendString("\r\n");
}

/* !STOP — 停止 */
static void ble_handle_stop_command(void)
{
    APP_Control_StopDirect();
    BSP_UART_SendString("OK STOP\r\n");
}

/* !PID [kp] [ki] — 设置或查询 PI 参数 */
static void ble_handle_pid_command(const char *cmd)
{
    const char *p = cmd + 4;
    while (*p == ' ') p++;

    if (*p == '?') {
        /* Query mode */
        float kp, ki;
        APP_Control_GetPID(&kp, &ki);
        BSP_UART_SendString("OK PID:");
        ble_uart_send_float(kp, 1);
        BSP_UART_SendByte(',');
        ble_uart_send_float(ki, 1);
        BSP_UART_SendString("\r\n");
        return;
    }

    /* Set mode: !PID <kp> <ki> */
    {
        float kp = (float)atof(p);
        while (*p >= '0' && *p <= '9') p++;
        if (*p == '.') { p++; while (*p >= '0' && *p <= '9') p++; }
        while (*p == ' ') p++;
        float ki = (float)atof(p);

        APP_Control_SetPID(kp, ki);
        BSP_UART_SendString("OK PID:");
        ble_uart_send_float(kp, 1);
        BSP_UART_SendByte(',');
        ble_uart_send_float(ki, 1);
        BSP_UART_SendString("\r\n");
    }
}

/* ---- 命令分发 ---- */

static void ble_dispatch_command(void)
{
    /* !SPD */
    if (s_line_len >= 5 &&
        s_line_buf[0] == '!' && s_line_buf[1] == 'S' &&
        s_line_buf[2] == 'P' && s_line_buf[3] == 'D') {
        ble_handle_spd_command((const char *)s_line_buf);
        return;
    }
    /* !DRIVE */
    if (s_line_len >= 6 &&
        s_line_buf[1] == 'D' && s_line_buf[2] == 'R' &&
        s_line_buf[3] == 'I') {
        ble_handle_drive_command((const char *)s_line_buf);
        return;
    }
    /* !PID */
    if (s_line_len >= 4 &&
        s_line_buf[1] == 'P' && s_line_buf[2] == 'I' &&
        s_line_buf[3] == 'D') {
        ble_handle_pid_command((const char *)s_line_buf);
        return;
    }
    /* !SWP */
    if (s_line_len >= 4 &&
        s_line_buf[1] == 'S' && s_line_buf[2] == 'W' &&
        s_line_buf[3] == 'P') {
        ble_handle_swp_command((const char *)s_line_buf);
        return;
    }
    /* !STOP */
    if (s_line_len >= 5 &&
        s_line_buf[1] == 'S' && s_line_buf[2] == 'T' &&
        s_line_buf[3] == 'O') {
        ble_handle_stop_command();
        return;
    }
    BSP_UART_SendString("?CMD\r\n");
}

/* ---- 公开 API ---- */

bool MID_BLE_Init(void)
{
    s_line_len = 0;

    s_spd_task.active     = false;
    s_spd_task.dumping    = false;
    s_spd_task.buf_count  = 0;
    s_spd_task.dump_index = 0;

    return true;
}

void MID_BLE_Poll(void)
{
    uint8_t byte;

    ble_spd_task_poll();

    /* Sweep data dump: when !SWP profile completes, send buffered data */
    if (APP_Control_IsSweepDone()) {
        uint16_t count = APP_Control_GetSweepCount();
        int16_t *target   = APP_Control_GetSweepTarget();
        int16_t *actual_l = APP_Control_GetSweepActualL();
        int16_t *actual_r = APP_Control_GetSweepActualR();

        for (uint16_t i = 0; i < count; i++) {
            BSP_UART_SendString("SWP:");
            ble_uart_send_int(target[i]);
            BSP_UART_SendByte(',');
            ble_uart_send_int(actual_l[i]);
            BSP_UART_SendByte(',');
            ble_uart_send_int(actual_r[i]);
            BSP_UART_SendString("\r\n");
        }
        BSP_UART_SendString("SWP_DONE\r\n");
    }

    while (BSP_UART_Available() > 0) {
        byte = BSP_UART_ReadByte();

        if (s_line_len < LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = byte;
        }
        if (byte == '\n' || byte == '\r') {
            s_line_buf[s_line_len] = '\0';
            if (s_line_len > 1) ble_dispatch_command();
            s_line_len = 0;
        }
    }
}

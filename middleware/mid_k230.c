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
 * mid_k230.c — K230 视觉模块协议解析
 *
 * 解析 K230 通过 UART1 发送的钢球位置数据。
 * K230 端 cx_to_mm() 输出单位为 mm。
 * 帧格式:
 *   检测到球:  "X:%+.1f,T:%u,D:1\r\n"     例: "X:+35.0,T:12345,D:1\r\n"
 *   未检测到:  "X:--,T:%u,D:0\r\n"        例: "X:--,T:12346,D:0\r\n"
 *
 * UART: 115200 bps, 8N1, PB7=RX ← K230 TX=11
 * 与 BLE 共享 UART1，分时复用。
 *
 * 依赖: bsp_uart, bsp_delay
 */

#include "mid_k230.h"
#include "bsp_uart.h"
#include "bsp_delay.h"
#include <stdlib.h>
#include <string.h>

/* ---- Constants ---- */
#define K230_LINE_BUF_SIZE  32   /* "X:-12.0,T:4294967295,D:0\r\n" ≈ 29 chars */

/* ---- Static state ---- */
static char     s_line_buf[K230_LINE_BUF_SIZE];
static uint8_t  s_line_pos         = 0;

static float    s_ball_position    = 0.0f;
static bool     s_ball_detected    = false;
static uint32_t s_k230_timestamp   = 0;    /* K230 帧时间戳 ms */
static uint32_t s_last_update      = 0;    /* 本地 tick — 用于超时检测 */

/* ---- Helpers ---- */

/*
 * Parse a complete line (no trailing \r \n).
 * Format: "X:<value>,T:<tick_ms>,D:<0|1>"
 * Returns true if line matched expected format.
 */
static bool k230_parse_line(const char *line)
{
    const char *p;

    /* Must start with "X:" */
    if (line[0] != 'X' || line[1] != ':')
        return false;

    p = line + 2;

    /* ---- Parse X field ---- */
    float    pos      = 0.0f;
    bool     detected = false;

    if (p[0] == '-' && p[1] == '-') {
        /* "--" : ball not detected */
        detected = false;
        p += 2;
    } else if (p[0] == '+' || p[0] == '-') {
        /* signed float, atof reads until first non-numeric char */
        pos      = (float)atof(p);
        detected = true;
        while (*p && *p != ',') p++;          /* skip float digits */
    } else {
        return false;                         /* unrecognized X field */
    }

    /* ---- Expect ",T:" ---- */
    if (p[0] != ',' || p[1] != 'T' || p[2] != ':')
        return false;
    p += 3;
    uint32_t k230_tick = (uint32_t)atoi(p);   /* atoi reads until non-digit */
    while (*p && *p != ',') p++;

    /* ---- Expect ",D:" ---- */
    if (p[0] != ',' || p[1] != 'D' || p[2] != ':')
        return false;
    p += 3;
    int d_flag = (*p == '1') ? 1 : 0;

    /* ---- Sanity: D flag must match X field ---- */
    if (detected  && d_flag != 1) return false;
    if (!detected && d_flag != 0) return false;

    /* ---- Store ---- */
    s_ball_position  = -pos;  /* K230 sign flipped, negate to match physical coordinate */
    s_ball_detected  = detected;
    s_k230_timestamp = k230_tick;
    s_last_update    = BSP_Delay_GetTick();
    return true;
}

/* ---- Public API ---- */

bool MID_K230_Init(void)
{
    s_line_pos        = 0;
    s_ball_position   = 0.0f;
    s_ball_detected   = false;
    s_k230_timestamp  = 0;
    s_last_update     = 0;
    return true;
}

/*
 * Call from main loop (non-blocking).
 * Reads available UART bytes, assembles lines, parses K230 frames.
 */
void MID_K230_Poll(void)
{
    while (BSP_UART_Available() > 0) {
        uint8_t c = BSP_UART_ReadByte();

        /* Ignore bare \r */
        if (c == '\r') {
            continue;
        }

        /* Line terminator */
        if (c == '\n') {
            if (s_line_pos > 0) {
                s_line_buf[s_line_pos] = '\0';
                k230_parse_line(s_line_buf);
                s_line_pos = 0;
            }
            continue;
        }

        /* Accumulate, with overflow protection */
        if (s_line_pos < K230_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_pos++] = (char)c;
        } else {
            /* Overflow: discard and resync */
            s_line_pos = 0;
        }
    }
}

bool MID_K230_IsDetected(void)
{
    return s_ball_detected;
}

float MID_K230_GetPosition(void)
{
    return s_ball_position;
}

uint32_t MID_K230_GetTimestamp(void)
{
    return s_k230_timestamp;
}

uint32_t MID_K230_GetLastUpdate(void)
{
    return s_last_update;
}

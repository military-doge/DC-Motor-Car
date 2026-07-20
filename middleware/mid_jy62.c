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

#include "mid_jy62.h"

/* State machine states */
#define STATE_WAIT_HEADER  0
#define STATE_WAIT_TYPE    1
#define STATE_READ_DATA    2
#define STATE_CHECKSUM     3

/* Frame type bytes */
#define FRAME_ACCEL  0x51
#define FRAME_GYRO   0x52
#define FRAME_ANGLE  0x53
#define FRAME_HEADER 0x55

/* Scale factors */
#define GYRO_SCALE   2000.0f
#define ANGLE_SCALE  180.0f
#define SCALE_DIV    32768.0f

static volatile uint8_t  s_state       = STATE_WAIT_HEADER;
static volatile uint8_t  s_data_idx    = 0;
static volatile uint8_t  s_frame_type  = 0;
static volatile uint8_t  s_buf[8];
static volatile uint8_t  s_checksum;

static volatile float    s_wz   = 0.0f;
static volatile float    s_roll = 0.0f;
static volatile float    s_pitch = 0.0f;
static volatile float    s_yaw  = 0.0f;
static volatile uint8_t  s_data_ok = 0;

void MID_JY62_Init(void)
{
    s_state      = STATE_WAIT_HEADER;
    s_data_idx   = 0;
    s_frame_type = 0;
    s_checksum   = 0;
    s_wz         = 0.0f;
    s_roll       = 0.0f;
    s_pitch      = 0.0f;
    s_yaw        = 0.0f;
    s_data_ok    = 0;
}

float MID_JY62_GetAngularVelocityZ(void)
{
    return s_wz;
}

float MID_JY62_GetYaw(void)
{
    return s_yaw;
}

float MID_JY62_GetRoll(void)
{
    return s_roll;
}

float MID_JY62_GetPitch(void)
{
    return s_pitch;
}

uint8_t MID_JY62_IsDataReady(void)
{
    return s_data_ok;
}

void MID_JY62_UartRxIsr(uint8_t byte)
{
    switch (s_state) {
    case STATE_WAIT_HEADER:
        if (byte == FRAME_HEADER) {
            s_checksum = byte;
            s_state = STATE_WAIT_TYPE;
        }
        break;

    case STATE_WAIT_TYPE:
        s_frame_type = byte;
        s_checksum += byte;
        if (byte == FRAME_ACCEL || byte == FRAME_GYRO || byte == FRAME_ANGLE) {
            s_data_idx = 0;
            s_state = STATE_READ_DATA;
        } else {
            s_state = STATE_WAIT_HEADER;
        }
        break;

    case STATE_READ_DATA:
        s_buf[s_data_idx] = byte;
        s_checksum += byte;
        s_data_idx++;
        if (s_data_idx >= 8) {
            s_state = STATE_CHECKSUM;
        }
        break;

    case STATE_CHECKSUM:
        /* Verify checksum: sum of first 10 bytes & 0xFF == byte */
        if ((s_checksum & 0xFF) == byte) {
            int16_t raw;

            switch (s_frame_type) {
            case FRAME_GYRO:
                raw = ((int16_t)s_buf[5] << 8) | s_buf[4];
                s_wz = (float)raw / SCALE_DIV * GYRO_SCALE;
                s_data_ok = 1;
                break;

            case FRAME_ANGLE:
                /* Data[0..1]=Roll, Data[2..3]=Pitch, Data[4..5]=Yaw */
                raw = ((int16_t)s_buf[1] << 8) | s_buf[0];
                s_roll  = (float)raw / SCALE_DIV * ANGLE_SCALE;
                raw = ((int16_t)s_buf[3] << 8) | s_buf[2];
                s_pitch = (float)raw / SCALE_DIV * ANGLE_SCALE;
                raw = ((int16_t)s_buf[5] << 8) | s_buf[4];
                s_yaw   = (float)raw / SCALE_DIV * ANGLE_SCALE;
                break;

            case FRAME_ACCEL:
                /* Acceleration frame — ignored */
                break;
            }
        }
        /* Checksum fail or processed: back to waiting for next frame */
        s_state = STATE_WAIT_HEADER;
        break;
    }
}

float MID_JY62_Normalize180(float angle)
{
    if (angle > 180.0f)  return angle - 360.0f;
    if (angle < -180.0f) return angle + 360.0f;
    return angle;
}

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

#include "mid_gyro_hold.h"
#include "mid_jy62.h"

#define K_YAW            0.005f
#define YAW_DEADZONE     2.0f
#define YAW_CORR_MAX     0.20f   /* Max correction to one wheel (m/s) */

static float   s_yaw_reference = 0.0f;
static uint8_t s_hold_active   = 0;

void MID_GyroHold_Init(void)
{
    s_yaw_reference = 0.0f;
    s_hold_active   = 0;
}

void MID_GyroHold_SetReference(void)
{
    if (!MID_JY62_IsDataReady()) {
        s_hold_active = 0;
        return;
    }
    s_yaw_reference = MID_JY62_GetYaw();
    s_hold_active   = 1;
}

void MID_GyroHold_Clear(void)
{
    s_hold_active = 0;
}

float MID_GyroHold_GetCorrection(void)
{
    float error;

    if (!s_hold_active || !MID_JY62_IsDataReady())
        return 0.0f;

    error = MID_JY62_GetYaw() - s_yaw_reference;
    error = MID_JY62_Normalize180(error);

    if (error < YAW_DEADZONE && error > -YAW_DEADZONE)
        return 0.0f;

    error = error * K_YAW;

    if (error > YAW_CORR_MAX)  error = YAW_CORR_MAX;
    if (error < -YAW_CORR_MAX) error = -YAW_CORR_MAX;

    return error;
}

float MID_GyroHold_GetError(void)
{
    if (!MID_JY62_IsDataReady())
        return 0.0f;

    return MID_JY62_Normalize180(MID_JY62_GetYaw() - s_yaw_reference);
}

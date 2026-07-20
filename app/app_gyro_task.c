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

#include "app_gyro_task.h"
#include "mid_jy62.h"
#include "mid_gyro_hold.h"

/* ---- Tunable parameters ---- */

#define GYRO_TASK_TURN_SPEED      0.10f   /* In-place rotation speed (m/s) */
#define GYRO_TASK_TURN_ANGLE      40.0f   /* Right-turn target angle (°) */
#define GYRO_TASK_ALIGN_THRESHOLD 3.0f    /* Angle alignment tolerance (°) */

/*
 * Distance per encoder pulse:
 *   perimeter = 0.2104867 m
 *   pulses/rev = 13 lines * 2 (quadrature) * 20 (gear ratio) = 520
 *   distance_per_pulse = 0.2104867 / 520 ≈ 0.00040478 m
 *
 * For 128 cm: target_pulses = 1.28 / 0.00040478 ≈ 3162 (per wheel)
 * Use sum of both wheels: ~6324 total pulses
 */
#define GYRO_TASK_PERIMETER       0.2104867f
#define GYRO_TASK_PULSES_PER_REV  520
#define GYRO_TASK_DRIVE_DIST_M    1.28f
#define GYRO_TASK_TARGET_PULSES   ((int32_t)(GYRO_TASK_DRIVE_DIST_M / \
    (GYRO_TASK_PERIMETER / GYRO_TASK_PULSES_PER_REV) * 2.0f))

/* ---- Static state ---- */

static app_gyro_task_state_t s_state = APP_GYRO_TASK_IDLE;
static float   s_reference_yaw  = 0.0f;
static float   s_target_yaw     = 0.0f;
static int32_t s_distance_sum   = 0;

/* ---- Public API ---- */

void APP_GyroTask_Init(void)
{
    s_state         = APP_GYRO_TASK_IDLE;
    s_reference_yaw = 0.0f;
    s_target_yaw    = 0.0f;
    s_distance_sum  = 0;
}

void APP_GyroTask_Start(void)
{
    /* Only start from idle */
    if (s_state != APP_GYRO_TASK_IDLE)
        return;

    if (!MID_JY62_IsDataReady())
        return;

    s_reference_yaw = MID_JY62_GetYaw();
    /* Right turn = CW rotation = yaw decreases */
    s_target_yaw    = MID_JY62_Normalize180(s_reference_yaw - GYRO_TASK_TURN_ANGLE);
    s_distance_sum  = 0;
    s_state         = APP_GYRO_TASK_TURN;
}

app_gyro_task_state_t APP_GyroTask_GetState(void)
{
    return s_state;
}

bool APP_GyroTask_Update(float drive_speed,
    int32_t encoder_a, int32_t encoder_b,
    float *left_tgt, float *right_tgt)
{
    float yaw_error;

    switch (s_state) {

    case APP_GYRO_TASK_IDLE:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return false;

    case APP_GYRO_TASK_TURN:
        if (!MID_JY62_IsDataReady()) {
            *left_tgt  = 0.0f;
            *right_tgt = 0.0f;
            return false;
        }

        yaw_error = MID_JY62_Normalize180(s_target_yaw - MID_JY62_GetYaw());

        if (yaw_error > -GYRO_TASK_ALIGN_THRESHOLD
            && yaw_error < GYRO_TASK_ALIGN_THRESHOLD) {
            /* Aligned — lock heading and start driving */
            MID_GyroHold_SetReference();
            s_distance_sum = 0;
            s_state = APP_GYRO_TASK_DRIVE;
            /* Fall through to DRIVE case */
        } else if (yaw_error > 0) {
            /* Target ahead — rotate CCW (left): left backward, right forward */
            *left_tgt  = -GYRO_TASK_TURN_SPEED;
            *right_tgt =  GYRO_TASK_TURN_SPEED;
            return false;
        } else {
            /* Target behind — rotate CW (right): left forward, right backward */
            *left_tgt  =  GYRO_TASK_TURN_SPEED;
            *right_tgt = -GYRO_TASK_TURN_SPEED;
            return false;
        }
        /* If we reach here (aligned), continue to DRIVE below */
        /* FALLTHROUGH */

    case APP_GYRO_TASK_DRIVE: {
        float correction;

        /* Accumulate absolute encoder pulses */
        s_distance_sum += (encoder_a > 0 ? encoder_a : -encoder_a)
                        + (encoder_b > 0 ? encoder_b : -encoder_b);

        if (s_distance_sum >= GYRO_TASK_TARGET_PULSES) {
            /* Distance reached — stop */
            *left_tgt  = 0.0f;
            *right_tgt = 0.0f;
            MID_GyroHold_Clear();
            s_state = APP_GYRO_TASK_DONE;
            return true;  /* Signal: task just finished */
        }

        /* Drive straight with gyro hold correction */
        correction = MID_GyroHold_GetCorrection();
        *left_tgt  = drive_speed + correction;
        *right_tgt = drive_speed - correction;
        return false;
    }

    case APP_GYRO_TASK_DONE:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return false;

    default:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return false;
    }
}

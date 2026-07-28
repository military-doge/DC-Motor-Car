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

#include "mid_line_track.h"
#include <stdint.h>

/* ---- Tunable constants ---- */
/*
 * Black line is 2 sensors wide → sum=2, so error = weighted_sum/2.
 * Error range (all line widths 1-8): 0, ±1, ±2, ..., ±7.
 * Table size 8 covers all possible |error| (0..7).
 *
 * Three lookup tables indexed by abs_error:
 *   s_attenuation_table — speed attenuation ratio (0.0~1.0), from desktop
 *       tuned values. base_speed = MID_TRACK_BASE_SPEED * ratio[idx].
 *   s_kp_table   — proportional gain, from desktop tuned values
 *   s_kd_table   — derivative gain, from desktop tuned values */
#define MID_TRACK_TBL_SIZE        8
#define MID_TRACK_BASE_SPEED      0.50f   /* straight-line speed (m/s) */

static const float s_attenuation_table[MID_TRACK_TBL_SIZE] = {
    1.00f,    /* err=0  — 100%  (straight)    */
    0.95f,    /* err=1  — 95.0%               */
    0.82f,    /* err=2  — 82.0%               */
    0.62f,    /* err=3  — 62.0%               */
    0.47f,    /* err=4  — 47.0%               */
    0.37f,    /* err=5  — 37.0%               */
    0.28f,    /* err=6  — 28.0%               */
    0.22f,    /* err=7  — 22.0%               */
};
static const float s_kp_table[MID_TRACK_TBL_SIZE] = {
    [0] = 0.0058f, [1] = 0.0090f, [2] = 0.0144f, [3] = 0.0140f,
    [4] = 0.0114f, [5] = 0.0090f, [6] = 0.0148f, [7] = 0.0132f,
};
static const float s_kd_table[MID_TRACK_TBL_SIZE] = {
    [0] = 0.0040f, [1] = 0.0060f, [2] = 0.0090f, [3] = 0.0090f,
    [4] = 0.0070f, [5] = 0.0060f, [6] = 0.0095f, [7] = 0.0090f,
};

/* 8-channel sensor weights (asymmetric, left-to-right) */
#define MID_TRACK_W0  (-10)
#define MID_TRACK_W1  (-6)
#define MID_TRACK_W2  (-3)
#define MID_TRACK_W3  (-1)
#define MID_TRACK_W4  (1)
#define MID_TRACK_W5  (3)
#define MID_TRACK_W6  (6)
#define MID_TRACK_W7  (10)

static const int8_t s_track_weights[8] = {
    MID_TRACK_W0, MID_TRACK_W1, MID_TRACK_W2, MID_TRACK_W3,
    MID_TRACK_W4, MID_TRACK_W5, MID_TRACK_W6, MID_TRACK_W7
};

#define MID_TRACK_SEARCH_SPEED  0.28f
#define MID_TRACK_SEARCH_KP     0.038f
#define MID_TRACK_SEARCH_CAP    6

/* ---- Control state ---- */
static int8_t s_line_error       = 0;
static int8_t s_prev_error       = 0;
static int8_t s_last_valid_error = 0;

bool MID_LineTrack_Init(void)
{
    s_line_error       = 0;
    s_prev_error       = 0;
    s_last_valid_error = 0;
    return true;
}

void MID_LineTrack_Reset(void)
{
    s_line_error       = 0;
    s_prev_error       = 0;
    s_last_valid_error = 0;
}

int8_t MID_LineTrack_GetError(void)
{
    return s_line_error;
}

/*
 * Nonlinear PD line-tracking update. Called every 10ms from timer ISR
 * via the app control callback.
 *
 * - Weighted average of 8 binary sensors produces error (-7..+7)
 * - No deadzone: continuous correction at all errors
 * - Nonlinear speed decay: base = MID_TRACK_BASE_SPEED × attenuation ratio
 * - P+D: standard PD, KD tuned from KP up
 * - Differential steering: left = base + correction, right = base - correction
 *
 * Special cases:
 *   All 8 black (intersection) → drive straight
 *   All white (line lost)       → turn toward last known line direction
 */
void MID_LineTrack_Update(const uint16_t sensor_data[8],
    float *out_left_speed, float *out_right_speed)
{
    int weighted_sum, sum, error, abs_error;
    float correction, derivative, base_speed;

    /* Step 1: weighted average to compute line position error */
    weighted_sum = 0;
    sum = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (sensor_data[i]) {
            weighted_sum += s_track_weights[i];
            sum++;
        }
    }

    /* Intersection → drive straight */
    if (sum == 8) {
        s_line_error  = 0;
        *out_left_speed  = MID_TRACK_BASE_SPEED;
        *out_right_speed = MID_TRACK_BASE_SPEED;
        return;
    }

    /* Line lost → turn toward last known direction */
    if (sum == 0) {
        s_line_error = 0;
        if (s_last_valid_error != 0) {
            int dir = (s_last_valid_error > 0) ? 1 : -1;
            int mag = (s_last_valid_error > 0) ? s_last_valid_error : -s_last_valid_error;
            if (mag > MID_TRACK_SEARCH_CAP) mag = MID_TRACK_SEARCH_CAP;
            float search_correction = MID_TRACK_SEARCH_KP * (float)(dir * mag);
            *out_left_speed  = MID_TRACK_SEARCH_SPEED + search_correction;
            *out_right_speed = MID_TRACK_SEARCH_SPEED - search_correction;
        } else {
            *out_left_speed  = MID_TRACK_BASE_SPEED;
            *out_right_speed = MID_TRACK_BASE_SPEED;
        }
        return;
    }

    s_line_error = weighted_sum / sum;  /* range: -7 .. +7 */
    s_last_valid_error = s_line_error;

    error     = s_line_error;
    abs_error = (error > 0) ? error : -error;

    /* Step 2: base speed = straight speed × attenuation ratio */
    uint8_t idx = (abs_error < MID_TRACK_TBL_SIZE) ? abs_error : (MID_TRACK_TBL_SIZE - 1);
    base_speed   = MID_TRACK_BASE_SPEED * s_attenuation_table[idx];
    float kp_eff = s_kp_table[idx];
    float kd_eff = s_kd_table[idx];

    /* Step 5: error derivative */
    derivative = (float)(error - s_prev_error);
    s_prev_error = error;

    /* Step 6: P + D */
    correction = kp_eff * (float)error
               + kd_eff * derivative;

    /* Step 6: differential steering output */
    *out_left_speed  = base_speed + correction;
    *out_right_speed = base_speed - correction;
}

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
 * Two lookup tables indexed by abs_error:
 *   s_kp_table   — proportional gain, from desktop tuned values
 *   s_kd_table   — derivative gain, from desktop tuned values */
#define MID_TRACK_TBL_SIZE        8
#define MID_TRACK_BASE_SPEED      0.35f   /* straight-line speed (m/s) */

static const float s_kp_table[MID_TRACK_TBL_SIZE] = {
    [0] = 0.0080f, [1] = 0.0120f, [2] = 0.0180f, [3] = 0.0180f,
    [4] = 0.0204f, [5] = 0.0187f, [6] = 0.0170f, [7] = 0.0152f,
};
static const float s_kd_table[MID_TRACK_TBL_SIZE] = {
    [0] = 0.0047f, [1] = 0.0059f, [2] = 0.0075f, [3] = 0.0075f,
    [4] = 0.0064f, [5] = 0.0059f, [6] = 0.0075f, [7] = 0.0075f,
};

/* 8-channel sensor weights (asymmetric, left-to-right) */
#define MID_TRACK_W0  (-7)
#define MID_TRACK_W1  (-5)
#define MID_TRACK_W2  (-3)
#define MID_TRACK_W3  (-1)
#define MID_TRACK_W4  (1)
#define MID_TRACK_W5  (3)
#define MID_TRACK_W6  (5)
#define MID_TRACK_W7  (7)

static const int8_t s_track_weights[8] = {
    MID_TRACK_W0, MID_TRACK_W1, MID_TRACK_W2, MID_TRACK_W3,
    MID_TRACK_W4, MID_TRACK_W5, MID_TRACK_W6, MID_TRACK_W7
};

#define MID_TRACK_SEARCH_SPEED  0.15f
#define MID_TRACK_SEARCH_KP     0.038f
#define MID_TRACK_SEARCH_CAP    6

/* ---- Control state ---- */
static int8_t s_line_error       = 0;
static int8_t s_prev_error       = 0;
static int8_t s_last_valid_error = 0;
static int8_t s_filtered_error   = 0;
static bool  s_line_lost         = false;

bool MID_LineTrack_Init(void)
{
    s_line_error       = 0;
    s_prev_error       = 0;
    s_last_valid_error = 0;
    s_filtered_error   = 0;
    s_line_lost        = false;
    return true;
}

void MID_LineTrack_Reset(void)
{
    s_line_error       = 0;
    s_prev_error       = 0;
    s_last_valid_error = 0;
    s_filtered_error   = 0;
    s_line_lost        = false;
}

int8_t MID_LineTrack_GetError(void)
{
    return s_line_error;
}

int8_t MID_LineTrack_GetLastError(void)
{
    return s_last_valid_error;
}

bool MID_LineTrack_IsLineLost(void)
{
    return s_line_lost;
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
        s_line_lost   = false;
        *out_left_speed  = MID_TRACK_BASE_SPEED;
        *out_right_speed = MID_TRACK_BASE_SPEED;
        return;
    }

    /* Line lost → turn toward last known direction */
    if (sum == 0) {
        s_line_error = 0;
        s_line_lost  = true;
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
    s_line_lost = false;

    /* Slew rate limit: clamp error change to ±5 per tick */
    {
        int8_t delta = s_line_error - s_filtered_error;
        if (delta > 5) delta = 5;
        if (delta < -5) delta = -5;
        s_filtered_error += delta;
    }

    error     = s_filtered_error;
    abs_error = (error > 0) ? error : -error;

    /* Step 2: uniform base speed, KP/KD from table */
    uint8_t idx = (abs_error < MID_TRACK_TBL_SIZE) ? abs_error : (MID_TRACK_TBL_SIZE - 1);
    base_speed   = MID_TRACK_BASE_SPEED;
    float kp_eff = s_kp_table[idx];
    float kd_eff = s_kd_table[idx];

    /* Step 5: error derivative */
    derivative = (float)(error - s_prev_error);
    s_prev_error = error;

    /* Step 6: P + D */
    correction = kp_eff * (float)error
               + kd_eff * derivative;

    /* Step 6.5: clamp correction to prevent inner wheel reversal */
    if (correction > base_speed) correction = base_speed;
    if (correction < -base_speed) correction = -base_speed;

    /* Step 7: differential steering output */
    *out_left_speed  = base_speed + correction;
    *out_right_speed = base_speed - correction;
}

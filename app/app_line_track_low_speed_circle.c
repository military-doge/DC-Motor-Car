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

#include "app_line_track_low_speed_circle.h"
#include <math.h>
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"
#include "mid_line_track.h"

/* ---- Constants ---- */
#define APP_LTC_FREQ             100.0f
#define APP_LTC_PERIMETER        0.2042f
#define APP_LTC_PULSES_PER_REV   25525
#define APP_LTC_SPEED_ALPHA      0.4f
#define APP_LTC_DEADBAND         0.005f
#define APP_LTC_PWM_MAX          7800

/* ---- Acceleration profile ----
 * Phase 1: 0 → 6.14m uniform acceleration from rest (a ≈ 0.0144 m/s^2)
 * Phase 2: 6.14m → 6.20m constant speed (v_max = 0.42 m/s)
 * Constraint: avg speed over first 6.14m = 0.21 m/s
 *   v_avg = v_max / 2 = 0.21  →  v_max = 0.42 m/s
 *   a = v_max^2 / (2 * 6.14) = 0.1764 / 12.28 ≈ 0.01436 m/s^2
 *   t_accel ≈ 29.2 s,  t_total_6.2m ≈ 29.4 s
 * Track perimeter ≈ 2×1.5 + π×1.0 ≈ 6.14m (one full lap)
 */
#define APP_LTC_ACCEL_DIST_M     6.14f
#define APP_LTC_ACCEL_MPS2       (0.1764f / 12.28f)
#define APP_LTC_VMAX_MPS         0.42f

/* Minimum speed floor to overcome static friction at dead start. */
#define APP_LTC_VMIN_MPS         0.08f

/* Distance: 6.2m auto-stop */
#define APP_LTC_DIST_PER_PULSE   (APP_LTC_PERIMETER / APP_LTC_PULSES_PER_REV / 2.0f)
#define APP_LTC_TARGET_DIST_M    6.20f
#define APP_LTC_TARGET_PULSES    ((int32_t)(APP_LTC_TARGET_DIST_M / APP_LTC_DIST_PER_PULSE))

/* ---- Speed PI ---- */
#define APP_LTC_KP  600.0f
#define APP_LTC_KI  600.0f

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;
    float target_speed;
    float pwm_output;
} app_ltc_motor_t;

/* ---- Static state ---- */
static volatile app_ltc_motor_t s_motor_left;
static volatile app_ltc_motor_t s_motor_right;
static volatile bool s_running      = false;
static volatile bool s_done         = false;
static volatile uint32_t s_tick     = 0;
static volatile uint16_t s_sensor_data[8];

/* Distance */
static volatile int32_t s_total_pulses = 0;
static volatile float   s_distance_m   = 0.0f;

/* PI accumulator */
static float s_last_bias_left;
static float s_last_bias_right;

/* ---- Helpers ---- */

static float app_ltc_pwm_limit(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

static int16_t app_ltc_pi_update(float current, float target, float *last_bias,
    float *pwm)
{
    float bias = target - current;
    float abs_bias = (bias > 0.0f) ? bias : -bias;

    if (abs_bias < APP_LTC_DEADBAND) {
        *last_bias = bias;
        return (int16_t)(*pwm);
    }

    *pwm += APP_LTC_KP * (bias - *last_bias) + APP_LTC_KI * bias;
    *last_bias = bias;
    *pwm = app_ltc_pwm_limit(*pwm, (float)(-APP_LTC_PWM_MAX), (float)APP_LTC_PWM_MAX);

    return (int16_t)(*pwm);
}

static float app_ltc_calc_speed(int16_t encoder_count)
{
    return (float)encoder_count * APP_LTC_FREQ * APP_LTC_PERIMETER
        / (float)APP_LTC_PULSES_PER_REV;
}

static float app_ltc_lowpass(float raw, float *filtered)
{
    *filtered = APP_LTC_SPEED_ALPHA * raw + (1.0f - APP_LTC_SPEED_ALPHA) * (*filtered);
    return *filtered;
}

static void app_ltc_fmt_speed(char *buf, float speed_mps)
{
    int16_t cms = (int16_t)(speed_mps * 100.0f);
    uint8_t pos = 0;
    if (cms < 0) {
        buf[pos++] = '-';
        cms = -cms;
    } else {
        buf[pos++] = '+';
    }
    buf[pos++] = '0' + (cms / 100) % 10;
    buf[pos++] = '0' + (cms / 10) % 10;
    buf[pos++] = '0' + cms % 10;
    buf[pos]   = '\0';
}

/* Format distance as "X.XX" meters */
static void app_ltc_fmt_dist(char *buf, float dist_m)
{
    int32_t cm = (int32_t)(dist_m * 100.0f);
    uint8_t pos = 0;
    buf[pos++] = '0' + (cm / 100) % 10;
    buf[pos++] = '.';
    buf[pos++] = '0' + (cm / 10) % 10;
    buf[pos++] = '0' + cm % 10;
    buf[pos]   = '\0';
}

/* ---- Speed profile: acceleration → constant ---- */
static float app_ltc_calc_target_speed(float dist_m)
{
    float v;
    if (dist_m <= 0.0f) {
        return APP_LTC_VMIN_MPS;
    }
    if (dist_m < APP_LTC_ACCEL_DIST_M) {
        /* Uniform acceleration: v = sqrt(2 * a * d) */
        v = sqrtf(2.0f * APP_LTC_ACCEL_MPS2 * dist_m);
        if (v < APP_LTC_VMIN_MPS) {
            v = APP_LTC_VMIN_MPS;
        }
        return (v < APP_LTC_VMAX_MPS) ? v : APP_LTC_VMAX_MPS;
    }
    return APP_LTC_VMAX_MPS;
}

/* ---- Public API ---- */

void APP_LineTrack_LowSpeedCircle_Init(void)
{
    s_motor_left.current_speed  = 0.0f;
    s_motor_left.target_speed   = 0.0f;
    s_motor_left.pwm_output     = 0.0f;
    s_motor_right.current_speed = 0.0f;
    s_motor_right.target_speed  = 0.0f;
    s_motor_right.pwm_output    = 0.0f;
    s_running      = false;
    s_done         = false;
    s_tick         = 0;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;

    /* Set initial base speed (starts from VMIN, ramps up per acceleration profile) */
    MID_LineTrack_SetBaseSpeed(0.0f);
}

/*
 * Called from 10ms timer ISR callback.
 * Reads sensors -> MID_LineTrack PD -> reads encoders -> speed calc -> PI -> PWM.
 */
void APP_LineTrack_LowSpeedCircle_TimerTick(void)
{
    int16_t count_a, count_b;
    float raw_a, raw_b;
    int16_t pwm_a, pwm_b;
    uint16_t raw_sensor[8];
    float left_tgt, right_tgt;
    uint8_t i;

    /* 1. LED */
    if (s_running) {
        BSP_LED_Flash(100);
    }

    /* 2. Key scan */
    BSP_Key_Scan();

    /* 3. Read grayscale sensors */
    BSP_Grayscale_ReadAll(raw_sensor);

    /* Store for display */
    for (i = 0; i < 8; i++) {
        s_sensor_data[i] = raw_sensor[i];
    }

    /* 4. Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* 5. Convert to raw speed (m/s) */
    raw_a = app_ltc_calc_speed(count_a);
    raw_b = app_ltc_calc_speed(-count_b);

    /* 6. Low-pass filter */
    app_ltc_lowpass(raw_a, &s_motor_left.current_speed);
    app_ltc_lowpass(raw_b, &s_motor_right.current_speed);

    /* 7. Odometry */
    s_total_pulses += (count_a > 0 ? count_a : -count_a)
                    + (count_b > 0 ? count_b : -count_b);
    s_distance_m = (float)s_total_pulses * APP_LTC_DIST_PER_PULSE;

    /* 7.5 Auto-stop at 6.2m */
    if (s_total_pulses >= APP_LTC_TARGET_PULSES) {
        s_running = false;
        s_done    = true;
        s_motor_left.target_speed  = 0.0f;
        s_motor_right.target_speed = 0.0f;
        BSP_Motor_Stop();
        return;
    }

    /* 8. Stop check */
    if (!s_running) {
        BSP_Motor_Stop();
        return;
    }

    /* 7.8 Update base speed from acceleration profile */
    MID_LineTrack_SetBaseSpeed(app_ltc_calc_target_speed(s_distance_m));

    /* 8. Compute line-tracking targets from middleware */
    MID_LineTrack_Update(raw_sensor, &left_tgt, &right_tgt);

    s_motor_left.target_speed  = left_tgt;
    s_motor_right.target_speed = right_tgt;

    /* 9. PI control and PWM output */
    pwm_a = app_ltc_pi_update(s_motor_left.current_speed,
        s_motor_left.target_speed, &s_last_bias_left,
        &s_motor_left.pwm_output);
    pwm_b = app_ltc_pi_update(s_motor_right.current_speed,
        s_motor_right.target_speed, &s_last_bias_right,
        &s_motor_right.pwm_output);
    BSP_Motor_SetPWM(pwm_a, pwm_b);

    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout (12px font, 128x64):
 *   LOW CIR +030
 *   L:+020 R:+020
 *   D:1.23/6.20m
 *   RUN
 */
void APP_LineTrack_LowSpeedCircle_Run(void)
{
    char buf[5];

    if (s_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    if (s_done) {
        /* Finished: 6.2m reached */
        MID_OLED_ShowString(18, 0, "6.2M DONE!", 12);
        {
            char dbuf[5];
            app_ltc_fmt_dist(dbuf, s_distance_m);
            MID_OLED_ShowString(0, 20, "Dist:", 12);
            MID_OLED_ShowString(42, 20, dbuf, 12);
            MID_OLED_ShowString(90, 20, "m", 12);
        }
        {
            float avg = s_distance_m / ((float)s_tick * 0.01f);
            char sbuf[5];
            app_ltc_fmt_speed(sbuf, avg);
            sbuf[4] = '\0';
            MID_OLED_ShowString(0, 40, "Avg:", 12);
            MID_OLED_ShowString(36, 40, sbuf, 12);
            MID_OLED_ShowString(66, 40, "cm/s", 12);
        }
    } else if (!s_running) {
        MID_OLED_ShowString(12, 0, "LOW SPD CIR", 12);
        MID_OLED_ShowString(36, 24, "READY", 12);
        MID_OLED_ShowString(12, 40, "Key=Start", 12);
    } else {
        /* Line 0: title with dynamic target speed */
        {
            char tbuf[5];
            app_ltc_fmt_speed(tbuf, app_ltc_calc_target_speed(s_distance_m));
            MID_OLED_ShowString(6, 0, "LOW CIR ", 12);
            MID_OLED_ShowString(72, 0, tbuf, 12);
        }

        /* Line 16: L actual speed */
        app_ltc_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf, 12);

        /* Line 28: R actual speed */
        app_ltc_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(0, 28, "R:", 12);
        MID_OLED_ShowString(12, 28, buf, 12);

        /* Line 40: distance */
        {
            char dbuf[5];
            app_ltc_fmt_dist(dbuf, s_distance_m);
            MID_OLED_ShowString(0, 40, "D:", 12);
            MID_OLED_ShowString(12, 40, dbuf, 12);
            MID_OLED_ShowString(54, 40, "/6.20m", 12);
        }

        /* Line 52: status */
        MID_OLED_ShowString(0, 52, "RUN", 12);
    }

    MID_OLED_RefreshGram();
}

void APP_LineTrack_LowSpeedCircle_Start(void)
{
    MID_LineTrack_Reset();
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;
    s_total_pulses = 0;
    s_distance_m   = 0.0f;
    s_done         = false;
    s_running      = true;
}

void APP_LineTrack_LowSpeedCircle_Stop(void)
{
    s_running = false;
    s_done    = false;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    BSP_Motor_Stop();
}

bool APP_LineTrack_LowSpeedCircle_IsRunning(void)
{
    return s_running;
}

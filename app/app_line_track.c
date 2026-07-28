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

#include "app_line_track.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"
#include "mid_line_track.h"

/* ---- Physical constants ---- */
#define APP_LT_FREQ             100.0f
#define APP_LT_PERIMETER        0.2042f     /* wheel circumference: 65mm * pi */
#define APP_LT_PULSES_PER_REV   25525       /* calibrated encoder pulses per wheel rev */
#define APP_LT_SPEED_ALPHA      0.4f        /* low-pass filter coefficient */
#define APP_LT_DEADBAND         0.005f      /* PI deadband (m/s) */
#define APP_LT_PWM_MAX          7800

/* ---- Speed PI parameters (from README) ---- */
#define APP_LT_KP  600.0f
#define APP_LT_KI  800.0f

/* ---- Line-loss recovery phases (consecutive 10ms ticks) ---- */
#define APP_LT_LOST_PHASE1      20    /* 0-200ms: trust mid_line_track search */
#define APP_LT_LOST_PHASE2      50    /* 200-500ms: pivot turn toward last dir */
                                       /* >500ms: full spin toward last dir */

#define APP_LT_PIVOT_SPEED      0.07f /* pivot turn speed (m/s) */
#define APP_LT_SPIN_SPEED       0.11f /* full spin speed (m/s) */

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;
    float target_speed;
    float pwm_output;
} app_lt_motor_t;

/* ---- Static state ---- */
static volatile app_lt_motor_t s_motor_left;
static volatile app_lt_motor_t s_motor_right;
static volatile bool s_running      = false;
static volatile uint32_t s_tick     = 0;
static volatile uint16_t s_sensor_data[8];

/* PI accumulator */
static float s_last_bias_left;
static float s_last_bias_right;

/* Line-loss recovery */
static volatile uint16_t s_lost_ticks = 0;

/* ---- Helpers ---- */

static float app_lt_pwm_limit(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

/* Incremental discrete PI controller with deadband */
static int16_t app_lt_pi_update(float current, float target, float *last_bias,
    float *pwm)
{
    float bias = target - current;
    float abs_bias = (bias > 0.0f) ? bias : -bias;

    if (abs_bias < APP_LT_DEADBAND) {
        *last_bias = bias;
        return (int16_t)(*pwm);
    }

    *pwm += APP_LT_KP * (bias - *last_bias) + APP_LT_KI * bias;
    *last_bias = bias;
    *pwm = app_lt_pwm_limit(*pwm, (float)(-APP_LT_PWM_MAX), (float)APP_LT_PWM_MAX);

    return (int16_t)(*pwm);
}

/* Encoder counts to m/s */
static float app_lt_calc_speed(int16_t encoder_count)
{
    return (float)encoder_count * APP_LT_FREQ * APP_LT_PERIMETER
        / (float)APP_LT_PULSES_PER_REV;
}

/* Low-pass filter */
static float app_lt_lowpass(float raw, float *filtered)
{
    *filtered = APP_LT_SPEED_ALPHA * raw + (1.0f - APP_LT_SPEED_ALPHA) * (*filtered);
    return *filtered;
}

/* Convert m/s to 4-char signed string in cm/s, e.g. "+010", "-003" */
static void app_lt_fmt_speed(char *buf, float speed_mps)
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

/* ---- Public API ---- */

void APP_LineTrack_Init(void)
{
    s_motor_left.current_speed  = 0.0f;
    s_motor_left.target_speed   = 0.0f;
    s_motor_left.pwm_output     = 0.0f;
    s_motor_right.current_speed = 0.0f;
    s_motor_right.target_speed  = 0.0f;
    s_motor_right.pwm_output    = 0.0f;
    s_running      = false;
    s_tick         = 0;
    s_lost_ticks   = 0;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
}

/*
 * Called from 10ms timer ISR callback.
 * Full line-tracking control loop:
 *   read sensors -> compute targets (MID_LineTrack) -> enhanced recovery
 *   -> read encoders -> calc speed -> lowpass -> PI -> PWM
 */
void APP_LineTrack_TimerTick(void)
{
    int16_t count_a, count_b;
    float raw_a, raw_b;
    int16_t pwm_a, pwm_b;
    uint16_t raw_sensor[8];
    float left_tgt, right_tgt;
    uint8_t sensor_cnt = 0;
    uint8_t i;

    /* 1. LED: fast blink when running */
    if (s_running) {
        BSP_LED_Flash(100);
    }

    /* 1.5 Key scan (must come early for precise 10ms timing) */
    BSP_Key_Scan();

    /* 2. Read grayscale sensors */
    BSP_Grayscale_ReadAll(raw_sensor);

    /* Count active sensors for line-loss detection */
    for (i = 0; i < 8; i++) {
        if (raw_sensor[i]) sensor_cnt++;
    }

    /* Store for display */
    for (i = 0; i < 8; i++) {
        s_sensor_data[i] = raw_sensor[i];
    }

    /* 3. Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* 4. Convert to raw speed (m/s) */
    raw_a = app_lt_calc_speed(count_a);
    raw_b = app_lt_calc_speed(-count_b);  /* Motor B is mechanically reversed */

    /* 5. Low-pass filter */
    app_lt_lowpass(raw_a, &s_motor_left.current_speed);
    app_lt_lowpass(raw_b, &s_motor_right.current_speed);

    /* 6. Stop: zero targets, stop motors */
    if (!s_running) {
        BSP_Motor_Stop();
        s_lost_ticks = 0;
        return;
    }

    /* 7. Compute line-tracking target speeds from middleware */
    MID_LineTrack_Update(raw_sensor, &left_tgt, &right_tgt);

    /* 8. Enhanced line-loss recovery — override targets when line is lost */
    if (MID_LineTrack_IsLineLost()) {
        s_lost_ticks++;

        if (s_lost_ticks > APP_LT_LOST_PHASE2) {
            /*
             * Phase 3 (>500ms): full spin toward last known direction.
             * One wheel forward, one backward — rotates in place.
             */
            int8_t last_err = MID_LineTrack_GetLastError();
            if (last_err > 0) {
                /* Line was to the right — spin right */
                left_tgt  =  APP_LT_SPIN_SPEED;
                right_tgt = -APP_LT_SPIN_SPEED;
            } else if (last_err < 0) {
                /* Line was to the left — spin left */
                left_tgt  = -APP_LT_SPIN_SPEED;
                right_tgt =  APP_LT_SPIN_SPEED;
            } else {
                /* No prior direction — spin right as default */
                left_tgt  =  APP_LT_SPIN_SPEED;
                right_tgt = -APP_LT_SPIN_SPEED;
            }
        } else if (s_lost_ticks > APP_LT_LOST_PHASE1) {
            /*
             * Phase 2 (200-500ms): pivot turn.
             * One wheel slow forward, one slow backward.
             */
            int8_t last_err = MID_LineTrack_GetLastError();
            if (last_err > 0) {
                left_tgt  =  APP_LT_PIVOT_SPEED;
                right_tgt = -APP_LT_PIVOT_SPEED;
            } else if (last_err < 0) {
                left_tgt  = -APP_LT_PIVOT_SPEED;
                right_tgt =  APP_LT_PIVOT_SPEED;
            } else {
                left_tgt  =  APP_LT_PIVOT_SPEED;
                right_tgt = -APP_LT_PIVOT_SPEED;
            }
        }
        /* Phase 1 (0-200ms): trust MID_LineTrack_Update's built-in search */
    } else {
        s_lost_ticks = 0;
    }

    s_motor_left.target_speed  = left_tgt;
    s_motor_right.target_speed = right_tgt;

    /* 9. PI control and PWM output */
    pwm_a = app_lt_pi_update(s_motor_left.current_speed,
        s_motor_left.target_speed, &s_last_bias_left,
        &s_motor_left.pwm_output);
    pwm_b = app_lt_pi_update(s_motor_right.current_speed,
        s_motor_right.target_speed, &s_last_bias_right,
        &s_motor_right.pwm_output);
    BSP_Motor_SetPWM(pwm_a, pwm_b);

    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout:
 *   S:n Er:±x       (sensor count, line error)
 *   L:+010|+009     (target | actual, cm/s)
 *   R:+010|+009
 *   Status: RUN / LOST / STOP
 */
void APP_LineTrack_Run(void)
{
    char buf[5];
    uint8_t cnt = 0;
    uint8_t i;

    if (s_tick % 50 != 0) {
        return;
    }

    /* Line-loss fast refresh: every 100ms when searching */
    if (s_lost_ticks > 0 && (s_tick % 10 != 0)) {
        return;
    }

    MID_OLED_Clear();

    if (!s_running) {
        MID_OLED_ShowString(48, 24, "STOP", 12);
    } else {
        for (i = 0; i < 8; i++) {
            if (s_sensor_data[i]) cnt++;
        }

        /* Line 0: sensor count + line error */
        {
            char buf_cnt[2];
            buf_cnt[0] = '0' + cnt;
            buf_cnt[1] = '\0';
            MID_OLED_ShowString(0, 0, "S:", 12);
            MID_OLED_ShowString(12, 0, buf_cnt, 12);
        }
        {
            int8_t error = MID_LineTrack_GetError();
            MID_OLED_ShowString(30, 0, "Er:", 12);
            if (error < 0) {
                MID_OLED_ShowString(54, 0, "-", 12);
                MID_OLED_ShowNumber(62, 0, (uint32_t)(-error), 2, 12);
            } else {
                MID_OLED_ShowString(54, 0, "+", 12);
                MID_OLED_ShowNumber(62, 0, (uint32_t)error, 2, 12);
            }
        }

        /* Line 16: L target */
        app_lt_fmt_speed(buf, s_motor_left.target_speed);
        MID_OLED_ShowString(0, 16, "Lt:", 12);
        MID_OLED_ShowString(18, 16, buf, 12);

        /* Line 16 (right half): L actual */
        app_lt_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(66, 16, buf, 12);

        /* Line 28: R target */
        app_lt_fmt_speed(buf, s_motor_right.target_speed);
        MID_OLED_ShowString(0, 28, "Rt:", 12);
        MID_OLED_ShowString(18, 28, buf, 12);

        /* Line 28 (right half): R actual */
        app_lt_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(66, 28, buf, 12);

        /* Line 44: status */
        if (s_lost_ticks > 0) {
            MID_OLED_ShowString(0, 44, "LOST ", 12);
            MID_OLED_ShowNumber(30, 44, s_lost_ticks, 3, 12);
            if (s_lost_ticks > APP_LT_LOST_PHASE2) {
                MID_OLED_ShowString(54, 44, "SPIN", 12);
            } else if (s_lost_ticks > APP_LT_LOST_PHASE1) {
                MID_OLED_ShowString(54, 44, "PIVOT", 12);
            } else {
                MID_OLED_ShowString(54, 44, "SRCH", 12);
            }
        } else {
            MID_OLED_ShowString(0, 44, "RUN", 12);
        }
    }

    MID_OLED_RefreshGram();
}

void APP_LineTrack_Start(void)
{
    MID_LineTrack_Reset();
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;
    s_lost_ticks = 0;
    s_running = true;
}

void APP_LineTrack_Stop(void)
{
    s_running = false;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    s_lost_ticks = 0;
    BSP_Motor_Stop();
}

bool APP_LineTrack_IsRunning(void)
{
    return s_running;
}

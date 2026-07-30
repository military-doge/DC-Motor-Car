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

#include "app_line_track_high_speed.h"
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

/* distance_m = sum_pulses * DIST_PER_SUM_PULSE
 * DIST_PER_SUM_PULSE = perimeter / pulses_per_rev / 2 = 0.000004 */
#define APP_LT_DIST_PER_PULSE   (APP_LT_PERIMETER / APP_LT_PULSES_PER_REV / 2.0f)

/* ---- Speed PI parameters ---- */
#define APP_LT_KP  600.0f
#define APP_LT_KI  600.0f

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

/* ---- Line-track state machine ---- */
typedef enum {
    APP_LT_IDLE = 0,
    APP_LT_RUNNING,
    APP_LT_DONE
} app_lt_state_t;

/* ---- Static state ---- */
static volatile app_lt_motor_t s_motor_left;
static volatile app_lt_motor_t s_motor_right;
static volatile bool s_running      = false;
static volatile uint32_t s_tick     = 0;
static volatile uint16_t s_sensor_data[8];

/* Distance & timing */
static volatile int32_t  s_total_pulses   = 0;
static volatile float    s_distance_m     = 0.0f;
static volatile uint32_t s_elapsed_ticks  = 0;      /* 10ms ticks since start */
static volatile app_lt_state_t s_state    = APP_LT_IDLE;

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

/* Convert m/s to 4-char signed string in cm/s, e.g. "+035", "-012" */
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

/* Format elapsed time as "XX.XX" seconds */
static void app_lt_fmt_time(char *buf, uint32_t ticks)
{
    uint32_t cs = ticks;  /* centiseconds (10ms ticks) */
    uint32_t sec = cs / 100;
    uint32_t frac = cs % 100;
    buf[0] = '0' + (sec / 10) % 10;
    buf[1] = '0' + sec % 10;
    buf[2] = '.';
    buf[3] = '0' + (frac / 10) % 10;
    buf[4] = '0' + frac % 10;
    buf[5] = '\0';
}

/* Format distance as "X.XX" meters */
static void app_lt_fmt_dist(char *buf, float dist_m)
{
    int32_t cm = (int32_t)(dist_m * 100.0f);
    uint8_t pos = 0;
    buf[pos++] = '0' + (cm / 100) % 10;
    buf[pos++] = '.';
    buf[pos++] = '0' + (cm / 10) % 10;
    buf[pos++] = '0' + cm % 10;
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
    s_total_pulses  = 0;
    s_distance_m    = 0.0f;
    s_elapsed_ticks = 0;
    s_state         = APP_LT_IDLE;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
}

/*
 * Called from 10ms timer ISR callback.
 * Full line-tracking control loop:
 *   read sensors -> compute targets (MID_LineTrack) -> enhanced recovery
 *   -> read encoders -> calc speed -> lowpass -> odometry -> PI -> PWM
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

    /* 6. When stopped, hold target 0 for active braking via PI */
    if (!s_running) {
        s_motor_left.target_speed  = 0.0f;
        s_motor_right.target_speed = 0.0f;
        s_lost_ticks = 0;
    }

    /* 7. Odometry & line tracking (only when running) */
    if (s_running) {
        /* Stopwatch: only tick while running, stops on forced-stop */
        s_elapsed_ticks++;

        /* Odometry: accumulate absolute encoder pulses */
        s_total_pulses += (count_a > 0 ? count_a : -count_a)
                        + (count_b > 0 ? count_b : -count_b);
        s_distance_m = (float)s_total_pulses * APP_LT_DIST_PER_PULSE;

        /* Auto-stop: any 4 consecutive channels all black */
        {
            uint8_t j;
            bool stop_now = false;
            for (j = 0; j <= 4; j++) {
                if (raw_sensor[j] && raw_sensor[j+1] && raw_sensor[j+2] && raw_sensor[j+3]) {
                    stop_now = true;
                    break;
                }
            }
            if (stop_now) {
                s_state = APP_LT_DONE;
                s_running = false;
                s_motor_left.target_speed  = 0.0f;
                s_motor_right.target_speed = 0.0f;
            }
        }

        if (s_running) {
            /* Compute line-tracking target speeds from middleware */
            MID_LineTrack_Update(raw_sensor, &left_tgt, &right_tgt);

            /* Enhanced line-loss recovery */
            if (MID_LineTrack_IsLineLost()) {
                s_lost_ticks++;

                if (s_lost_ticks > APP_LT_LOST_PHASE2) {
                    /* Phase 3 (>500ms): full spin */
                    int8_t last_err = MID_LineTrack_GetLastError();
                    if (last_err > 0) {
                        left_tgt  =  APP_LT_SPIN_SPEED;
                        right_tgt = -APP_LT_SPIN_SPEED;
                    } else if (last_err < 0) {
                        left_tgt  = -APP_LT_SPIN_SPEED;
                        right_tgt =  APP_LT_SPIN_SPEED;
                    } else {
                        left_tgt  =  APP_LT_SPIN_SPEED;
                        right_tgt = -APP_LT_SPIN_SPEED;
                    }
                } else if (s_lost_ticks > APP_LT_LOST_PHASE1) {
                    /* Phase 2 (200-500ms): pivot turn */
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
            } else {
                s_lost_ticks = 0;
            }

            s_motor_left.target_speed  = left_tgt;
            s_motor_right.target_speed = right_tgt;
        }
    }

    /* 10. PI control and PWM output */
    pwm_a = app_lt_pi_update(s_motor_left.current_speed,
        s_motor_left.target_speed, &s_last_bias_left,
        &s_motor_left.pwm_output);
    pwm_b = app_lt_pi_update(s_motor_right.current_speed,
        s_motor_right.target_speed, &s_last_bias_right,
        &s_motor_right.pwm_output);
    BSP_Motor_SetPWM(pwm_a, pwm_b);

    /* 11. General tick counter (OLED refresh, blink, etc.) */
    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout:
 *   [T] 12.34s           (elapsed time)
 *   L:+035|+035          (target | actual speed, cm/s)
 *   R:+035|+035
 *   D:3.05/6.14m  RUN    (distance / status)
 */
void APP_LineTrack_Run(void)
{
    char buf[6];

    if (s_tick % 50 != 0) {
        return;
    }

    /* Line-loss fast refresh: every 100ms */
    if (s_lost_ticks > 0 && (s_tick % 10 != 0)) {
        return;
    }

    MID_OLED_Clear();

    if (s_state == APP_LT_IDLE) {
        MID_OLED_ShowString(30, 0, "LINE-TRACK", 12);
        MID_OLED_ShowString(36, 20, "READY", 12);
        MID_OLED_ShowString(12, 36, "Key=Start", 12);
    } else if (s_state == APP_LT_DONE) {
        /* Finished: show result */
        MID_OLED_ShowString(18, 0, "LAP DONE!", 12);

        app_lt_fmt_time(buf, s_elapsed_ticks);
        MID_OLED_ShowString(0, 16, "Time:", 12);
        MID_OLED_ShowString(42, 16, buf, 12);
        MID_OLED_ShowString(90, 16, "s", 12);

        app_lt_fmt_dist(buf, s_distance_m);
        MID_OLED_ShowString(0, 32, "Dist:", 12);
        MID_OLED_ShowString(42, 32, buf, 12);
        MID_OLED_ShowString(90, 32, "m", 12);

        /* Average speed */
        {
            float avg = s_distance_m / ((float)s_elapsed_ticks * 0.01f);
            app_lt_fmt_speed(buf, avg);
            buf[4] = '\0'; /* truncate to "+035" format */
            MID_OLED_ShowString(0, 48, "Avg:", 12);
            MID_OLED_ShowString(36, 48, buf, 12);
            MID_OLED_ShowString(66, 48, "cm/s", 12);
        }
    } else {
        /* Running */
        /* Line 0: elapsed time */
        app_lt_fmt_time(buf, s_elapsed_ticks);
        MID_OLED_ShowString(0, 0, "T", 12);
        MID_OLED_ShowString(12, 0, buf, 12);
        MID_OLED_ShowString(66, 0, "s", 12);

        /* Line 12: L speed */
        app_lt_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 12, "L:", 12);
        MID_OLED_ShowString(12, 12, buf, 12);

        /* Line 24: R speed */
        app_lt_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(0, 24, "R:", 12);
        MID_OLED_ShowString(12, 24, buf, 12);

        /* Line 36: distance + status */
        app_lt_fmt_dist(buf, s_distance_m);
        MID_OLED_ShowString(0, 40, "D:", 12);
        MID_OLED_ShowString(12, 40, buf, 12);
        MID_OLED_ShowString(54, 40, "m", 12);

        /* Line 52: status */
        if (s_lost_ticks > 0) {
            MID_OLED_ShowString(0, 52, "LOST", 12);
            MID_OLED_ShowNumber(30, 52, s_lost_ticks, 3, 12);
        } else {
            MID_OLED_ShowString(0, 52, "RUN", 12);
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
    s_lost_ticks    = 0;
    s_total_pulses  = 0;
    s_distance_m    = 0.0f;
    s_elapsed_ticks = 0;
    s_state         = APP_LT_RUNNING;
    s_running       = true;
    MID_LineTrack_ResetParams();  /* restore full base speed */
}

void APP_LineTrack_Stop(void)
{
    s_running = false;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    s_lost_ticks = 0;
    if (s_state != APP_LT_DONE) {
        s_state = APP_LT_IDLE;  /* manual stop: reset */
    }
}

bool APP_LineTrack_IsRunning(void)
{
    return s_running;
}

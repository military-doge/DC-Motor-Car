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

#include "app_gyro_path.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"
#include "mid_jy62.h"
#include "mid_gyro_hold.h"

/* ---- Physical constants ---- */
#define APP_GP_FREQ             100.0f
#define APP_GP_PERIMETER        0.2042f     /* wheel circumference: 65mm * pi */
#define APP_GP_PULSES_PER_REV   25525       /* calibrated encoder pulses per wheel rev */
#define APP_GP_SPEED_ALPHA      0.4f        /* low-pass filter coefficient */
#define APP_GP_DEADBAND         0.005f      /* PI deadband (m/s) */
#define APP_GP_PWM_MAX          7800

/* ---- Speed PI parameters ---- */
#define APP_GP_KP  600.0f
#define APP_GP_KI  800.0f

/* ---- Path plan parameters ---- */
#define APP_GP_DRIVE_SPEED      0.20f   /* Straight-line drive speed (m/s) */
#define APP_GP_TURN_SPEED       0.10f   /* In-place rotation speed (m/s) */
#define APP_GP_TURN_ANGLE       90.0f   /* Right-turn target angle (degrees) */
#define APP_GP_ALIGN_THRESHOLD  3.0f    /* Angle alignment tolerance (degrees) */

/*
 * Distance per encoder pulse (calibrated):
 *   perimeter = 0.2042 m
 *   pulses/rev = 25525
 *   distance_per_pulse = 0.2042 / 25525 ≈ 0.00000800 m
 *
 * For 0.75 m: target_pulses ≈ 93750 per wheel, ~187500 sum of both
 */
#define APP_GP_DRIVE_DIST_M    0.75f
#define APP_GP_TARGET_PULSES   ((int32_t)(APP_GP_DRIVE_DIST_M / \
    (APP_GP_PERIMETER / APP_GP_PULSES_PER_REV) * 2.0f))

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;
    float target_speed;
    float pwm_output;
} app_gp_motor_t;

/* ---- Static state ---- */
static volatile app_gp_motor_t s_motor_left;
static volatile app_gp_motor_t s_motor_right;
static volatile bool s_running    = false;
static volatile uint32_t s_tick   = 0;
static volatile app_gyro_path_state_t s_state = APP_GYRO_PATH_IDLE;
static volatile float s_reference_yaw = 0.0f;
static volatile float s_target_yaw    = 0.0f;
static volatile int32_t s_distance_sum = 0;

/* PI accumulator */
static float s_last_bias_left;
static float s_last_bias_right;

/* ---- Helpers ---- */

static float app_gp_pwm_limit(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

/* Incremental discrete PI controller with deadband */
static int16_t app_gp_pi_update(float current, float target, float *last_bias,
    float *pwm)
{
    float bias = target - current;
    float abs_bias = (bias > 0.0f) ? bias : -bias;

    if (abs_bias < APP_GP_DEADBAND) {
        *last_bias = bias;
        return (int16_t)(*pwm);
    }

    *pwm += APP_GP_KP * (bias - *last_bias) + APP_GP_KI * bias;
    *last_bias = bias;
    *pwm = app_gp_pwm_limit(*pwm, (float)(-APP_GP_PWM_MAX), (float)APP_GP_PWM_MAX);

    return (int16_t)(*pwm);
}

/* Encoder counts to m/s */
static float app_gp_calc_speed(int16_t encoder_count)
{
    return (float)encoder_count * APP_GP_FREQ * APP_GP_PERIMETER
        / (float)APP_GP_PULSES_PER_REV;
}

/* Low-pass filter */
static float app_gp_lowpass(float raw, float *filtered)
{
    *filtered = APP_GP_SPEED_ALPHA * raw + (1.0f - APP_GP_SPEED_ALPHA) * (*filtered);
    return *filtered;
}

/* Convert m/s to 4-char signed string in cm/s, e.g. "+010", "-003" */
static void app_gp_fmt_speed(char *buf, float speed_mps)
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

/* ---- State-machine: compute target speeds from gyro + encoder odometry ---- */

static void app_gp_path_update(float *left_tgt, float *right_tgt)
{
    float yaw_error;
    float correction;

    switch (s_state) {

    case APP_GYRO_PATH_IDLE:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return;

    case APP_GYRO_PATH_DRIVE1:
        correction = MID_GyroHold_GetCorrection();
        *left_tgt  = APP_GP_DRIVE_SPEED + correction;
        *right_tgt = APP_GP_DRIVE_SPEED - correction;
        return;

    case APP_GYRO_PATH_TURN:
        if (!MID_JY62_IsDataReady()) {
            *left_tgt  = 0.0f;
            *right_tgt = 0.0f;
            return;
        }

        yaw_error = MID_JY62_Normalize180(s_target_yaw - MID_JY62_GetYaw());

        if (yaw_error > -APP_GP_ALIGN_THRESHOLD
            && yaw_error < APP_GP_ALIGN_THRESHOLD) {
            /* Aligned — lock heading and start second drive */
            MID_GyroHold_SetReference();
            s_distance_sum = 0;
            s_state = APP_GYRO_PATH_DRIVE2;
            /* Fall through to DRIVE2 */
        } else if (yaw_error > 0) {
            /* Target CCW: left backward, right forward */
            *left_tgt  = -APP_GP_TURN_SPEED;
            *right_tgt =  APP_GP_TURN_SPEED;
            return;
        } else {
            /* Target CW: left forward, right backward */
            *left_tgt  =  APP_GP_TURN_SPEED;
            *right_tgt = -APP_GP_TURN_SPEED;
            return;
        }
        /* FALLTHROUGH */

    case APP_GYRO_PATH_DRIVE2:
        correction = MID_GyroHold_GetCorrection();
        *left_tgt  = APP_GP_DRIVE_SPEED + correction;
        *right_tgt = APP_GP_DRIVE_SPEED - correction;
        return;

    case APP_GYRO_PATH_DONE:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return;

    default:
        *left_tgt  = 0.0f;
        *right_tgt = 0.0f;
        return;
    }
}

/* ---- Public API ---- */

void APP_GyroPath_Init(void)
{
    s_motor_left.current_speed  = 0.0f;
    s_motor_left.target_speed   = 0.0f;
    s_motor_left.pwm_output     = 0.0f;
    s_motor_right.current_speed = 0.0f;
    s_motor_right.target_speed  = 0.0f;
    s_motor_right.pwm_output    = 0.0f;
    s_running      = false;
    s_tick         = 0;
    s_state        = APP_GYRO_PATH_IDLE;
    s_reference_yaw = 0.0f;
    s_target_yaw    = 0.0f;
    s_distance_sum  = 0;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
}

/*
 * Called from 10ms timer ISR callback (100 Hz).
 * Full gyro-path control loop:
 *   Key scan → read encoders → calc speed → lowpass → path update → PI → PWM.
 * On state transitions, handles distance check and turn setup.
 */
void APP_GyroPath_TimerTick(void)
{
    int16_t count_a, count_b;
    float raw_a, raw_b;
    int16_t pwm_a, pwm_b;
    float left_tgt, right_tgt;

    /* 1. LED: fast blink when running */
    if (s_running) {
        BSP_LED_Flash(100);
    }

    /* 2. Key scan (precise 10ms timing) */
    BSP_Key_Scan();

    /* 3. Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* 4. Convert to raw speed (m/s) */
    raw_a = app_gp_calc_speed(count_a);
    raw_b = app_gp_calc_speed(-count_b);  /* Motor B is mechanically reversed */

    /* 5. Low-pass filter */
    app_gp_lowpass(raw_a, &s_motor_left.current_speed);
    app_gp_lowpass(raw_b, &s_motor_right.current_speed);

    /* 6. Stop: zero targets, stop motors */
    if (!s_running) {
        BSP_Motor_Stop();
        return;
    }

    /* 7. Distance accumulation (for DRIVE1 and DRIVE2 phases) */
    if (s_state == APP_GYRO_PATH_DRIVE1 || s_state == APP_GYRO_PATH_DRIVE2) {
        s_distance_sum += (count_a > 0 ? count_a : -count_a)
                        + (count_b > 0 ? count_b : -count_b);
    }

    /* 8. Check DRIVE1 → TURN transition */
    if (s_state == APP_GYRO_PATH_DRIVE1
        && s_distance_sum >= APP_GP_TARGET_PULSES) {
        s_reference_yaw = MID_JY62_GetYaw();
        s_target_yaw    = MID_JY62_Normalize180(s_reference_yaw - APP_GP_TURN_ANGLE);
        s_state         = APP_GYRO_PATH_TURN;
    }

    /* 9. Check DRIVE2 → DONE transition */
    if (s_state == APP_GYRO_PATH_DRIVE2
        && s_distance_sum >= APP_GP_TARGET_PULSES) {
        MID_GyroHold_Clear();
        s_state = APP_GYRO_PATH_DONE;
        s_running = false;
    }

    /* 10. Compute target speeds from path state machine */
    app_gp_path_update(&left_tgt, &right_tgt);

    s_motor_left.target_speed  = left_tgt;
    s_motor_right.target_speed = right_tgt;

    /* 11. PI control and PWM output */
    if (s_running) {
        pwm_a = app_gp_pi_update(s_motor_left.current_speed,
            s_motor_left.target_speed, &s_last_bias_left,
            &s_motor_left.pwm_output);
        pwm_b = app_gp_pi_update(s_motor_right.current_speed,
            s_motor_right.target_speed, &s_last_bias_right,
            &s_motor_right.pwm_output);
        BSP_Motor_SetPWM(pwm_a, pwm_b);
    }

    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout:
 *   State: DRIVE1/TURN/DRIVE2/DONE
 *   L:+010  R:+010    (actual speeds, cm/s)
 *   Dist: XXXXX       (accumulated pulses)
 */
void APP_GyroPath_Run(void)
{
    char buf[5];
    const char *phase_str;

    if (s_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    if (!s_running && s_state == APP_GYRO_PATH_IDLE) {
        MID_OLED_ShowString(36, 24, "GYRO OK", 12);
    } else {
        /* Line 0: phase name */
        switch (s_state) {
        case APP_GYRO_PATH_DRIVE1: phase_str = "DRIVE1"; break;
        case APP_GYRO_PATH_TURN:   phase_str = "TURN";   break;
        case APP_GYRO_PATH_DRIVE2: phase_str = "DRIVE2"; break;
        case APP_GYRO_PATH_DONE:   phase_str = "DONE";   break;
        default:                   phase_str = "IDLE";   break;
        }
        MID_OLED_ShowString(0, 0, phase_str, 12);

        /* Line 16: L actual speed */
        app_gp_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf, 12);

        /* Line 16 (right half): R actual speed */
        app_gp_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(54, 16, "R:", 12);
        MID_OLED_ShowString(66, 16, buf, 12);

        /* Line 32: distance pulses */
        MID_OLED_ShowString(0, 32, "Sum:", 12);
        MID_OLED_ShowNumber(30, 32, (uint32_t)s_distance_sum, 6, 12);

        /* Line 44: target pulses */
        MID_OLED_ShowString(0, 44, "Tgt:", 12);
        MID_OLED_ShowNumber(30, 44, APP_GP_TARGET_PULSES, 6, 12);
    }

    MID_OLED_RefreshGram();
}

void APP_GyroPath_Start(void)
{
    if (s_state != APP_GYRO_PATH_IDLE && s_state != APP_GYRO_PATH_DONE)
        return;

    if (!MID_JY62_IsDataReady())
        return;

    /* Lock current heading for the first straight drive */
    MID_GyroHold_SetReference();
    s_reference_yaw = 0.0f;
    s_target_yaw    = 0.0f;
    s_distance_sum  = 0;
    s_state         = APP_GYRO_PATH_DRIVE1;

    /* Reset PI state */
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;

    s_running = true;
}

void APP_GyroPath_Stop(void)
{
    s_running = false;
    s_state = APP_GYRO_PATH_IDLE;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    s_distance_sum = 0;
    MID_GyroHold_Clear();
    BSP_Motor_Stop();
}

bool APP_GyroPath_IsRunning(void)
{
    return s_running;
}

app_gyro_path_state_t APP_GyroPath_GetState(void)
{
    return s_state;
}

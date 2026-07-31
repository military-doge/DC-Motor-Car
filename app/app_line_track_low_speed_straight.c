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

#include "app_line_track_low_speed_straight.h"
#include <math.h>
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_servo.h"
#include "mid_oled.h"
#include "mid_line_track.h"

/* ---- Constants ---- */
#define APP_LTS_FREQ             100.0f
#define APP_LTS_PERIMETER        0.2042f
#define APP_LTS_PULSES_PER_REV   25525
#define APP_LTS_SPEED_ALPHA      0.4f
#define APP_LTS_DEADBAND         0.005f
#define APP_LTS_PWM_MAX          7800

/* ---- Acceleration profile (triangular: accel + decel, no cruise) ----
 * Phase 1: 0 → 0.80m uniform acceleration from rest to v_max
 * Phase 2: 0.80m → 2.0m uniform deceleration to ~0
 * Constraint: avg speed over first 1.5m = 0.2 m/s
 *   t_1.5 = (2×0.8 + 2.4×(1-√(1-1.4/2.4))) / v_max = 2.4508/v_max
 *   v_avg = 1.5 / t_1.5 = 1.5×v_max/2.4508 = 0.2  →  v_max ≈ 0.327 m/s
 *   a_accel = v_max²/(2×0.8) ≈ 0.0667 m/s²
 *   a_decel = v_max²/(2×1.2) ≈ 0.0445 m/s²
 */
#define APP_LTS_ACCEL_DIST_M     0.80f
#define APP_LTS_DECEL_DIST_M     1.20f
#define APP_LTS_ACCEL_MPS2       0.0667f
#define APP_LTS_DECEL_MPS2       0.0445f
#define APP_LTS_VMAX_MPS         0.327f

/* Minimum speed floor to overcome static friction at dead start.
 * Without this, v=0 at d=0 → PI bias=0 → no PWM → car stays stuck. */
#define APP_LTS_VMIN_MPS         0.08f

/* Distance: 2m auto-stop */
#define APP_LTS_DIST_PER_PULSE   (APP_LTS_PERIMETER / APP_LTS_PULSES_PER_REV / 2.0f)
#define APP_LTS_TARGET_DIST_M    2.0f
#define APP_LTS_TARGET_PULSES    ((int32_t)(APP_LTS_TARGET_DIST_M / APP_LTS_DIST_PER_PULSE))

/* ---- Speed PI ---- */
#define APP_LTS_KP  600.0f
#define APP_LTS_KI  600.0f

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;
    float target_speed;
    float pwm_output;
} app_lts_motor_t;

/* ---- Static state ---- */
static volatile app_lts_motor_t s_motor_left;
static volatile app_lts_motor_t s_motor_right;
static volatile bool s_running      = false;
static volatile bool s_done         = false;
static volatile uint32_t s_tick     = 0;
static volatile uint32_t s_start_tick = 0;
static volatile uint16_t s_sensor_data[8];

/* Distance */
static volatile int32_t s_total_pulses = 0;
static volatile float   s_distance_m   = 0.0f;

/* PI accumulator */
static float s_last_bias_left;
static float s_last_bias_right;

/* Servo tilt state */
static float s_servo_angle = 135.0f;

/* ---- Helpers ---- */

static float app_lts_pwm_limit(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

static int16_t app_lts_pi_update(float current, float target, float *last_bias,
    float *pwm)
{
    float bias = target - current;
    float abs_bias = (bias > 0.0f) ? bias : -bias;

    if (abs_bias < APP_LTS_DEADBAND) {
        *last_bias = bias;
        return (int16_t)(*pwm);
    }

    *pwm += APP_LTS_KP * (bias - *last_bias) + APP_LTS_KI * bias;
    *last_bias = bias;
    *pwm = app_lts_pwm_limit(*pwm, (float)(-APP_LTS_PWM_MAX), (float)APP_LTS_PWM_MAX);

    return (int16_t)(*pwm);
}

static float app_lts_calc_speed(int16_t encoder_count)
{
    return (float)encoder_count * APP_LTS_FREQ * APP_LTS_PERIMETER
        / (float)APP_LTS_PULSES_PER_REV;
}

static float app_lts_lowpass(float raw, float *filtered)
{
    *filtered = APP_LTS_SPEED_ALPHA * raw + (1.0f - APP_LTS_SPEED_ALPHA) * (*filtered);
    return *filtered;
}

static void app_lts_fmt_speed(char *buf, float speed_mps)
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
static void app_lts_fmt_dist(char *buf, float dist_m)
{
    int32_t cm = (int32_t)(dist_m * 100.0f);
    uint8_t pos = 0;
    buf[pos++] = '0' + (cm / 100) % 10;
    buf[pos++] = '.';
    buf[pos++] = '0' + (cm / 10) % 10;
    buf[pos++] = '0' + cm % 10;
    buf[pos]   = '\0';
}

/* ---- Speed profile: acceleration → deceleration (triangular) ---- */
static float app_lts_calc_target_speed(float dist_m)
{
    float v;
    float d_decel_start = APP_LTS_ACCEL_DIST_M;

    if (dist_m <= 0.0f) {
        return APP_LTS_VMIN_MPS;
    }
    if (dist_m < d_decel_start) {
        /* Phase 1: uniform acceleration, v = sqrt(2 * a * d) */
        v = sqrtf(2.0f * APP_LTS_ACCEL_MPS2 * dist_m);
        if (v < APP_LTS_VMIN_MPS) {
            v = APP_LTS_VMIN_MPS;
        }
        return (v < APP_LTS_VMAX_MPS) ? v : APP_LTS_VMAX_MPS;
    }
    /* Phase 2: uniform deceleration, v = sqrt(v_max² - 2 * a_decel * (d - d_accel)) */
    {
        float d_into_decel = dist_m - d_decel_start;
        float v_sq = APP_LTS_VMAX_MPS * APP_LTS_VMAX_MPS
                     - 2.0f * APP_LTS_DECEL_MPS2 * d_into_decel;
        if (v_sq <= 0.0f) {
            return 0.0f;
        }
        v = sqrtf(v_sq);
        return v;
    }
}

/* ---- Servo tilt compensation ----
 *
 * Physics (measured kinematics, 2026-07-30):
 *   a_ball = 0.0110 × |Δθ_servo|  [m/s²]  (magnitude, from φ=0.064×Δθ and a=g×φ)
 *   Car accel:  a_accel = 0.0667 m/s²  →  Δθ = 6.1° to cancel exactly
 *   Car decel:  a_decel = 0.0445 m/s²  →  Δθ = 4.1° to cancel exactly
 * Add ~2° margin for pipe friction → steady-state: 8° accel, 6° decel.
 *
 * Direction (实测):
 *   Accel → ball lags backward → need ball→+cm → CCW 逆转 (angle decrease)
 *   Decel → ball lunges forward → need ball→-cm → CW 正转 (angle increase)
 */
#define SERVO_ACCEL_KICK_DEG    18.0f   /* initial kick CCW (break static friction) */
#define SERVO_ACCEL_KICK_TICKS  15      /* kick duration: 150ms */
#define SERVO_ACCEL_SETTLE_TICKS 5      /* settle from kick→hold: 50ms */
#define SERVO_ACCEL_HOLD_DEG    12.0f   /* steady compensation CCW */
#define SERVO_DECEL_HOLD_DEG    6.0f    /* steady compensation CW */
#define SERVO_DECEL_RAMP_M      0.10f   /* ramp-in distance at start of decel */
#define SERVO_TAPER_M           0.10f   /* taper-out distance before phase end */

static void app_lts_update_servo(float dist_m)
{
    float desired = 135.0f;

    if (!s_running) {
        s_servo_angle = 135.0f;
        BSP_Servo_SetAngle(135);
        return;
    }

    if (dist_m < APP_LTS_ACCEL_DIST_M) {
        /* === Acceleration phase: CCW 逆转 → ball → +cm === */
        uint32_t elapsed = s_tick - s_start_tick;
        float remaining = APP_LTS_ACCEL_DIST_M - dist_m;

        if (elapsed < SERVO_ACCEL_KICK_TICKS) {
            /* Fast kick to break static friction: 0 → 18° CCW */
            float frac = (float)elapsed / (float)SERVO_ACCEL_KICK_TICKS;
            desired = 135.0f - SERVO_ACCEL_KICK_DEG * frac;
        } else if (elapsed < SERVO_ACCEL_KICK_TICKS + SERVO_ACCEL_SETTLE_TICKS) {
            /* Quick settle: kick → hold over SETTLE_TICKS */
            float frac = (float)(elapsed - SERVO_ACCEL_KICK_TICKS)
                         / (float)SERVO_ACCEL_SETTLE_TICKS;
            desired = 135.0f - (SERVO_ACCEL_KICK_DEG
                       - (SERVO_ACCEL_KICK_DEG - SERVO_ACCEL_HOLD_DEG) * frac);
        } else if (remaining < SERVO_TAPER_M) {
            /* Taper to 0° near end of accel (last 10cm) */
            float frac = remaining / SERVO_TAPER_M;  /* 1 → 0 */
            desired = 135.0f - SERVO_ACCEL_HOLD_DEG * frac;
        } else {
            /* Steady compensation: hold ACCEL_HOLD CCW */
            desired = 135.0f - SERVO_ACCEL_HOLD_DEG;
        }
    } else if (dist_m < APP_LTS_TARGET_DIST_M) {
        /* === Deceleration phase: CW 正转 → ball → -cm === */
        float d_into = dist_m - APP_LTS_ACCEL_DIST_M;
        float remaining = APP_LTS_TARGET_DIST_M - dist_m;

        if (d_into < SERVO_DECEL_RAMP_M) {
            /* Smooth ramp into CW tilt over first 10cm */
            float frac = d_into / SERVO_DECEL_RAMP_M;
            desired = 135.0f + SERVO_DECEL_HOLD_DEG * frac;
        } else if (remaining < SERVO_TAPER_M) {
            /* Taper to 0° near end of decel (last 10cm) */
            float frac = remaining / SERVO_TAPER_M;
            desired = 135.0f + SERVO_DECEL_HOLD_DEG * frac;
        } else {
            /* Steady compensation: hold 6° CW */
            desired = 135.0f + SERVO_DECEL_HOLD_DEG;
        }
    }

    /* Slew rate limit: 3°/tick for smooth motion */
    {
        float diff = desired - s_servo_angle;
        float max_step = 3.0f;
        if (diff > max_step) diff = max_step;
        if (diff < -max_step) diff = -max_step;
        s_servo_angle += diff;
    }

    BSP_Servo_SetAngle((uint16_t)(s_servo_angle + 0.5f));
}

/* ---- Public API ---- */

void APP_LineTrack_LowSpeedStraight_Init(void)
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

    /* Set initial base speed (starts from 0, ramps up per acceleration profile) */
    MID_LineTrack_SetBaseSpeed(0.0f);

    /* Servo to center */
    s_servo_angle = 135.0f;
    BSP_Servo_SetAngle(135);
}

/*
 * Called from 10ms timer ISR callback.
 * Reads sensors -> MID_LineTrack PD -> reads encoders -> speed calc -> PI -> PWM.
 */
void APP_LineTrack_LowSpeedStraight_TimerTick(void)
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
    raw_a = app_lts_calc_speed(count_a);
    raw_b = app_lts_calc_speed(-count_b);

    /* 6. Low-pass filter */
    app_lts_lowpass(raw_a, &s_motor_left.current_speed);
    app_lts_lowpass(raw_b, &s_motor_right.current_speed);

    /* 7. Odometry */
    s_total_pulses += (count_a > 0 ? count_a : -count_a)
                    + (count_b > 0 ? count_b : -count_b);
    s_distance_m = (float)s_total_pulses * APP_LTS_DIST_PER_PULSE;

    /* 7.5 Auto-stop at 2m */
    if (s_total_pulses >= APP_LTS_TARGET_PULSES) {
        s_running = false;
        s_done    = true;
        s_motor_left.target_speed  = 0.0f;
        s_motor_right.target_speed = 0.0f;
        s_servo_angle = 135.0f;
        BSP_Servo_SetAngle(135);
        BSP_Motor_Stop();
        return;
    }

    /* 8. Stop check */
    if (!s_running) {
        s_servo_angle = 135.0f;
        BSP_Servo_SetAngle(135);
        BSP_Motor_Stop();
        return;
    }

    /* 7.8 Update base speed from acceleration profile */
    MID_LineTrack_SetBaseSpeed(app_lts_calc_target_speed(s_distance_m));

    /* 8. Compute line-tracking targets from middleware */
    MID_LineTrack_Update(raw_sensor, &left_tgt, &right_tgt);

    s_motor_left.target_speed  = left_tgt;
    s_motor_right.target_speed = right_tgt;

    /* 9. PI control and PWM output */
    pwm_a = app_lts_pi_update(s_motor_left.current_speed,
        s_motor_left.target_speed, &s_last_bias_left,
        &s_motor_left.pwm_output);
    pwm_b = app_lts_pi_update(s_motor_right.current_speed,
        s_motor_right.target_speed, &s_last_bias_right,
        &s_motor_right.pwm_output);
    BSP_Motor_SetPWM(pwm_a, pwm_b);

    /* 9.5 Servo tilt compensation */
    app_lts_update_servo(s_distance_m);

    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout (12px font, 128x64):
 *   LOW SPD STRAIGHT
 *   L:+020|+020
 *   R:+020|+020
 *   RUN
 */
void APP_LineTrack_LowSpeedStraight_Run(void)
{
    char buf[5];

    if (s_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    if (s_done) {
        /* Finished: 2m reached */
        MID_OLED_ShowString(18, 0, "2M DONE!", 12);
        {
            char dbuf[5];
            app_lts_fmt_dist(dbuf, s_distance_m);
            MID_OLED_ShowString(0, 20, "Dist:", 12);
            MID_OLED_ShowString(42, 20, dbuf, 12);
            MID_OLED_ShowString(90, 20, "m", 12);
        }
        {
            float elapsed = (float)(s_tick - s_start_tick) * 0.01f;
            float avg = s_distance_m / elapsed;
            char sbuf[5];
            char tbuf[8];
            uint16_t sec;
            uint16_t ds;
            app_lts_fmt_speed(sbuf, avg);
            sbuf[4] = '\0';
            MID_OLED_ShowString(0, 40, "Avg:", 12);
            MID_OLED_ShowString(36, 40, sbuf, 12);
            MID_OLED_ShowString(66, 40, "cm/s", 12);
            /* elapsed time */
            sec = (uint16_t)elapsed;
            ds  = (uint16_t)(elapsed * 10.0f) % 10;
            tbuf[0] = '0' + (sec / 10) % 10;
            tbuf[1] = '0' + sec % 10;
            tbuf[2] = '.';
            tbuf[3] = '0' + ds;
            tbuf[4] = 's';
            tbuf[5] = '\0';
            MID_OLED_ShowString(0, 52, "Time:", 12);
            MID_OLED_ShowString(42, 52, tbuf, 12);
        }
    } else if (!s_running) {
        MID_OLED_ShowString(12, 0, "LOW SPD STR", 12);
        MID_OLED_ShowString(36, 24, "READY", 12);
        MID_OLED_ShowString(12, 40, "Key=Start", 12);
    } else {
        /* Line 0: title with dynamic target speed */
        {
            char tbuf[5];
            app_lts_fmt_speed(tbuf, app_lts_calc_target_speed(s_distance_m));
            MID_OLED_ShowString(6, 0, "LOW SPD ", 12);
            MID_OLED_ShowString(72, 0, tbuf, 12);
        }

        /* Line 16: L actual speed */
        app_lts_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf, 12);

        /* Line 28: R actual speed */
        app_lts_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(0, 28, "R:", 12);
        MID_OLED_ShowString(12, 28, buf, 12);

        /* Line 40: distance */
        {
            char dbuf[5];
            app_lts_fmt_dist(dbuf, s_distance_m);
            MID_OLED_ShowString(0, 40, "D:", 12);
            MID_OLED_ShowString(12, 40, dbuf, 12);
            MID_OLED_ShowString(54, 40, "/2.00m", 12);
        }

        /* Line 52: status + elapsed time */
        {
            float elapsed = (float)(s_tick - s_start_tick) * 0.01f;
            char tbuf[8];
            uint16_t sec = (uint16_t)elapsed;
            uint16_t ds = (uint16_t)(elapsed * 10.0f) % 10;
            tbuf[0] = '0' + (sec / 10) % 10;
            tbuf[1] = '0' + sec % 10;
            tbuf[2] = '.';
            tbuf[3] = '0' + ds;
            tbuf[4] = 's';
            tbuf[5] = '\0';
            MID_OLED_ShowString(0, 52, "RUN T:", 12);
            MID_OLED_ShowString(42, 52, tbuf, 12);
        }
    }

    MID_OLED_RefreshGram();
}

void APP_LineTrack_LowSpeedStraight_Start(void)
{
    MID_LineTrack_Reset();
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;
    s_total_pulses = 0;
    s_distance_m   = 0.0f;
    s_done         = false;
    s_start_tick   = s_tick;
    s_running      = true;
}

void APP_LineTrack_LowSpeedStraight_Stop(void)
{
    s_running = false;
    s_done    = false;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    s_servo_angle = 135.0f;
    BSP_Servo_SetAngle(135);
    BSP_Motor_Stop();
}

bool APP_LineTrack_LowSpeedStraight_IsRunning(void)
{
    return s_running;
}

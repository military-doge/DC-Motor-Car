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
#include "bsp_servo.h"
#include "bsp_delay.h"
#include "mid_oled.h"
#include "mid_line_track.h"
#include "mid_k230.h"

/* ---- Basic constants ---- */
#define APP_LTC_FREQ             100.0f
#define APP_LTC_PERIMETER        0.2042f
#define APP_LTC_PULSES_PER_REV   25525
#define APP_LTC_SPEED_ALPHA      0.4f
#define APP_LTC_DEADBAND         0.005f
#define APP_LTC_PWM_MAX          7800

/* ---- Q5 Speed profile (trapezoidal: accel -> cruise -> decel) ----
 * Phase 1: 0 -> 1.20m  uniform acceleration from rest to v_cruise
 * Phase 2: 1.20m -> 6.20m  cruise at v_cruise
 * Phase 3: 6.20m -> 7.00m  uniform deceleration to 0
 * Constraint: avg speed over first 6.2m = 0.23 m/s
 *   t_6.2 = t_accel + t_cruise = 2.4/v + 5.0/v = 7.4/v
 *   v_avg = 6.2 / t_6.2 = 6.2*v/7.4 = 0.23  ->  v_cruise = 0.2745 m/s
 *   a_accel = v^2/(2*1.2) = 0.0314 m/s^2
 *   a_decel = v^2/(2*0.8) = 0.0471 m/s^2
 */
#define APP_LTC_ACCEL_DIST_M     1.20f
#define APP_LTC_DECEL_DIST_M     0.80f
#define APP_LTC_ACCEL_MPS2       0.0254f
#define APP_LTC_DECEL_MPS2       0.0382f
#define APP_LTC_VCRUISE_MPS      0.2471f
#define APP_LTC_VMIN_MPS         0.08f

/* Distance: 7m auto-stop (full lap + margin) */
#define APP_LTC_TARGET_DIST_M    7.00f
#define APP_LTC_DECEL_START_M    (APP_LTC_TARGET_DIST_M - APP_LTC_DECEL_DIST_M)

#define APP_LTC_DIST_PER_PULSE   (APP_LTC_PERIMETER / APP_LTC_PULSES_PER_REV / 2.0f)
#define APP_LTC_TARGET_PULSES    ((int32_t)(APP_LTC_TARGET_DIST_M / APP_LTC_DIST_PER_PULSE))

/* ---- Speed PI ---- */
#define APP_LTC_KP  600.0f
#define APP_LTC_KI  600.0f

/* ---- Q5 Vision PID (target = 0mm, ball at O) ----
 * Gains from Q3 stationary balancing, validated 2026-07-31.
 * KP/KD < 0 per README authority formula.
 */
#define SERVO_CENTER_DEG         135
#define VISION_KP                -2.885f
#define VISION_KI                -0.096f
#define VISION_KD                -2.290f
#define VISION_I_MAX             43.0f
#define VISION_I_ERR_THR         20.0f
#define VISION_DATA_TIMEOUT_MS   200
#define PID_CLAMP_DEG            29.3f

/* ---- Fine-tuning softener ---- */
#define FINE_ERR_THR_MM          20.0f
#define FINE_SCALE_MIN           0.35f

/* ---- Feedforward ----
 * Kinematics: a_ball = 0.0110 * |dtheta| m/s^2
 * Kff = 1/0.0110 = 90.9 deg/(m/s^2)
 * Sign: a_car > 0 -> ball lags -> need CCW (neg offset)
 *       ff = -Kff * a_car
 */
#define KFF_ACCEL                90.9f
#define FF_CLAMP_DEG             35.0f

/* ---- Stuck detection ---- */
#define STUCK_VEL_THR_MM_S       5.0f
#define STUCK_POS_THR_MM         20.0f
#define STUCK_BOOST_DEG          7.68f

/* ---- Servo post-stop: continue 5s after auto-stop ---- */
#define SERVO_POST_STOP_TICKS    500

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;
    float target_speed;
    float pwm_output;
} app_ltc_motor_t;

/* ---- Static state ---- */
static volatile app_ltc_motor_t s_motor_left;
static volatile app_ltc_motor_t s_motor_right;
static volatile bool s_running       = false;
static volatile bool s_done          = false;
static volatile uint32_t s_tick      = 0;
static volatile uint32_t s_start_tick = 0;
static volatile uint32_t s_stop_tick  = 0;
static volatile uint16_t s_sensor_data[8];

/* Distance */
static volatile int32_t s_total_pulses = 0;
static volatile float   s_distance_m   = 0.0f;

/* PI accumulator */
static float s_last_bias_left;
static float s_last_bias_right;

/* ---- Vision PID state ---- */
static float    s_prev_k230_pos;
static uint32_t s_prev_k230_ts;
static float    s_i_accum       = 0.0f;
static float    s_ball_velocity = 0.0f;
static float    s_vision_trim   = 0.0f;
static float    s_k230_pos      = 0.0f;
static float    s_k230_error    = 0.0f;
static bool     s_k230_has_prev = false;

/* Car acceleration for feedforward */
static float    s_prev_car_speed = 0.0f;
static float    s_car_accel      = 0.0f;

/* Servo angle (smoothed) */
static float    s_servo_angle_f = 135.0f;

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

/* ---- Speed profile: trapezoidal (accel -> cruise -> decel) ---- */
static float app_ltc_calc_target_speed(float dist_m)
{
    float v;

    if (dist_m <= 0.0f) {
        return APP_LTC_VMIN_MPS;
    }

    if (dist_m < APP_LTC_ACCEL_DIST_M) {
        /* Phase 1: uniform acceleration, v = sqrt(2 * a * d) */
        v = sqrtf(2.0f * APP_LTC_ACCEL_MPS2 * dist_m);
        if (v < APP_LTC_VMIN_MPS) {
            v = APP_LTC_VMIN_MPS;
        }
        return (v < APP_LTC_VCRUISE_MPS) ? v : APP_LTC_VCRUISE_MPS;
    }

    if (dist_m < APP_LTC_DECEL_START_M) {
        /* Phase 2: cruise at constant speed */
        return APP_LTC_VCRUISE_MPS;
    }

    /* Phase 3: uniform deceleration, v = sqrt(v_cruise^2 - 2 * a_decel * (d - d_decel_start)) */
    {
        float d_into = dist_m - APP_LTC_DECEL_START_M;
        float v_sq = APP_LTC_VCRUISE_MPS * APP_LTC_VCRUISE_MPS
                     - 2.0f * APP_LTC_DECEL_MPS2 * d_into;
        if (v_sq <= 0.0f) {
            return 0.0f;
        }
        v = sqrtf(v_sq);
        return v;
    }
}

/* ---- Vision PID + Feedforward servo control ---- */

/*
 * Compute vision PID trim + acceleration feedforward.
 * Ported from app_ball_ctrl_1.c with feedforward path.
 * Returns total trim deg, or feedforward-only if K230 data unavailable.
 */
static float compute_servo_trim(float clamp_deg)
{
    float pos, error_mm, dt, vel, abs_err;
    float p_term, d_term, trim;
    float ff_trim;
    uint32_t age, k230_ts;
    int32_t diff;

    /* ---- Acceleration feedforward (always active) ---- */
    ff_trim = -KFF_ACCEL * s_car_accel;
    if (ff_trim > +FF_CLAMP_DEG) ff_trim = +FF_CLAMP_DEG;
    if (ff_trim < -FF_CLAMP_DEG) ff_trim = -FF_CLAMP_DEG;

    /* ---- Check K230 data freshness ---- */
    age = BSP_Delay_GetTick() - MID_K230_GetLastUpdate();
    if (!(MID_K230_IsDetected() && (MID_K230_GetLastUpdate() > 0)
            && (age < VISION_DATA_TIMEOUT_MS))) {
        s_vision_trim = 0.0f;
        return ff_trim;
    }

    pos      = MID_K230_GetPosition();
    k230_ts  = MID_K230_GetTimestamp();
    error_mm = 0.0f - pos;  /* target = 0 (ball at O) */

    /* ---- dt from K230 timestamps, velocity (mm/s) ---- */
    diff = (int32_t)(k230_ts - s_prev_k230_ts);
    if (s_k230_has_prev && diff > 0 && diff < 500) {
        dt = (float)diff / 1000.0f;
        vel = (pos - s_prev_k230_pos) / dt;
        s_ball_velocity = vel;
        s_prev_k230_pos = pos;
        s_prev_k230_ts  = k230_ts;
    } else if (s_k230_has_prev) {
        vel = s_ball_velocity;
    } else {
        vel = 0.0f;
        s_ball_velocity = 0.0f;
        s_prev_k230_pos = pos;
        s_prev_k230_ts  = k230_ts;
        s_k230_has_prev = true;
    }

    s_k230_pos   = pos;
    s_k230_error = error_mm;

    /* ---- P term ---- */
    p_term = VISION_KP * error_mm;

    /* ---- I term (conditional accumulation + anti-windup) ---- */
    abs_err = error_mm < 0 ? -error_mm : error_mm;
    if (abs_err < VISION_I_ERR_THR) {
        s_i_accum += VISION_KI * error_mm * dt;
        if (s_i_accum > +VISION_I_MAX) s_i_accum = +VISION_I_MAX;
        if (s_i_accum < -VISION_I_MAX) s_i_accum = -VISION_I_MAX;
    }

    /* ---- D term (derivative-on-measurement) ---- */
    d_term = VISION_KD * (-vel);

    /* ---- Sum PID ---- */
    trim = p_term + s_i_accum + d_term;

    /* ---- Stuck detection ---- */
    {
        float abs_vel = vel < 0 ? -vel : vel;
        if (abs_vel < STUCK_VEL_THR_MM_S && abs_err > STUCK_POS_THR_MM
                && s_k230_has_prev) {
            trim += (error_mm > 0) ? -STUCK_BOOST_DEG : +STUCK_BOOST_DEG;
        }
    }

    /* ---- PID clamp ---- */
    if (trim > +clamp_deg) trim = +clamp_deg;
    if (trim < -clamp_deg) trim = -clamp_deg;

    /* ---- Fine-tuning softener ---- */
    {
        float abs_err_fine = error_mm < 0 ? -error_mm : error_mm;
        float scale;
        if (abs_err_fine >= FINE_ERR_THR_MM) {
            scale = 1.0f;
        } else {
            scale = FINE_SCALE_MIN
                  + (1.0f - FINE_SCALE_MIN) * (abs_err_fine / FINE_ERR_THR_MM);
        }
        trim *= scale;
    }

    s_vision_trim = trim;

    /* ---- PID + Feedforward ---- */
    trim += ff_trim;
    if (trim > +clamp_deg) trim = +clamp_deg;
    if (trim < -clamp_deg) trim = -clamp_deg;

    return trim;
}

static void app_ltc_update_servo(void)
{
    float desired;

    /* Keep servo active during running AND post-run 5s window */
    if (!s_running && !s_done) {
        s_servo_angle_f = 135.0f;
        s_i_accum       = 0.0f;
        BSP_Servo_SetAngle(135);
        return;
    }

    desired = 135.0f + compute_servo_trim(PID_CLAMP_DEG);

    /* Slew rate limit: 5 deg/tick for smooth motion */
    {
        float diff = desired - s_servo_angle_f;
        float max_step = 5.0f;
        if (diff > max_step) diff = max_step;
        if (diff < -max_step) diff = -max_step;
        s_servo_angle_f += diff;
    }

    BSP_Servo_SetAngle((uint16_t)(s_servo_angle_f + 0.5f));
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
    s_running       = false;
    s_done          = false;
    s_tick          = 0;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;

    /* Vision state */
    s_i_accum        = 0.0f;
    s_ball_velocity  = 0.0f;
    s_vision_trim    = 0.0f;
    s_k230_pos       = 0.0f;
    s_k230_error     = 0.0f;
    s_k230_has_prev  = false;
    s_prev_k230_pos  = 0.0f;
    s_prev_k230_ts   = 0;
    s_prev_car_speed = 0.0f;
    s_car_accel      = 0.0f;

    MID_LineTrack_SetBaseSpeed(0.0f);
    s_servo_angle_f = 135.0f;
    BSP_Servo_SetAngle(135);
}

/*
 * Called from 10ms timer ISR callback.
 * grayscale -> encoder -> speed -> PI -> PWM
 * -> vision PID + feedforward -> servo.
 * K230 is polled in main loop (main.c).
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

    /* 7.5 Auto-stop at target distance */
    if (s_total_pulses >= APP_LTC_TARGET_PULSES && s_running) {
        s_running = false;
        s_done    = true;
        s_stop_tick = s_tick;
        s_motor_left.target_speed  = 0.0f;
        s_motor_right.target_speed = 0.0f;
        BSP_Motor_Stop();
        /* Servo stays alive for 5s post-run — fall through */
    }

    /* 8. Manual stop check (key press, not auto-stop) */
    if (!s_running && !s_done) {
        s_servo_angle_f = 135.0f;
        BSP_Servo_SetAngle(135);
        BSP_Motor_Stop();
        return;
    }

    /* 8.5 Post-run servo timeout: 5s after auto-stop */
    if (s_done && (s_tick - s_stop_tick >= SERVO_POST_STOP_TICKS)) {
        s_servo_angle_f = 135.0f;
        BSP_Servo_SetAngle(135);
        return;
    }

    /* 7.8 Update base speed + line tracking (only while running) */
    if (s_running) {
        MID_LineTrack_SetBaseSpeed(app_ltc_calc_target_speed(s_distance_m));
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
    }

    /* 9.3 Car acceleration estimation (low-pass filtered) */
    {
        float car_speed = (s_motor_left.current_speed
                         + s_motor_right.current_speed) * 0.5f;
        float raw_accel = (car_speed - s_prev_car_speed) * APP_LTC_FREQ;
        s_car_accel = 0.3f * raw_accel + 0.7f * s_car_accel;
        s_prev_car_speed = car_speed;
    }

    /* 9.5 Servo: vision PID + feedforward (active until 5s post-run) */
    app_ltc_update_servo();

    s_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 */
void APP_LineTrack_LowSpeedCircle_Run(void)
{
    char buf[5];

    if (s_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    if (s_done) {
        /* Finished: full lap reached, servo runs 5s post-stop */
        {
            uint32_t post_ticks = s_tick - s_stop_tick;
            if (post_ticks < SERVO_POST_STOP_TICKS) {
                uint16_t remain_s = (uint16_t)(5 - post_ticks / 100);
                MID_OLED_ShowString(0, 0, "Q5 DONE S:", 12);
                MID_OLED_ShowNumber(72, 0, remain_s, 1, 12);
                MID_OLED_ShowString(84, 0, "s", 12);
            } else {
                MID_OLED_ShowString(0, 0, "Q5 FULL LAP!", 12);
            }
        }
        {
            char dbuf[5];
            app_ltc_fmt_dist(dbuf, s_distance_m);
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
            app_ltc_fmt_speed(sbuf, avg);
            sbuf[4] = '\0';
            MID_OLED_ShowString(0, 40, "Avg:", 12);
            MID_OLED_ShowString(36, 40, sbuf, 12);
            MID_OLED_ShowString(66, 40, "cm/s", 12);
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
        MID_OLED_ShowString(0, 0, "Q5 FULL LAP", 12);
        MID_OLED_ShowString(36, 24, "READY", 12);
        MID_OLED_ShowString(12, 40, "Key=Start", 12);
    } else {
        /* Line 0: title + target speed */
        {
            char tbuf[5];
            app_ltc_fmt_speed(tbuf, app_ltc_calc_target_speed(s_distance_m));
            MID_OLED_ShowString(0, 0, "Q5 ", 12);
            MID_OLED_ShowString(24, 0, tbuf, 12);
        }

        /* Line 16: L/R actual speed */
        app_ltc_fmt_speed(buf, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf, 12);

        app_ltc_fmt_speed(buf, s_motor_right.current_speed);
        MID_OLED_ShowString(54, 16, "R:", 12);
        MID_OLED_ShowString(66, 16, buf, 12);

        /* Line 28: K230 ball position + servo angle */
        {
            char pbuf[7];
            uint8_t p = 0;
            int32_t v;
            if (MID_K230_IsDetected()) {
                float pos = MID_K230_GetPosition();
                v = (int32_t)(pos >= 0 ? pos + 0.5f : pos - 0.5f);
            } else {
                v = 0;
            }
            MID_OLED_ShowString(0, 28, "B:", 12);
            pbuf[p++] = (v < 0) ? '-' : '+';
            if (v < 0) v = -v;
            pbuf[p++] = '0' + (v / 10) % 10;
            pbuf[p++] = '0' + (v % 10);
            pbuf[p++] = 'm'; pbuf[p++] = 'm'; pbuf[p] = '\0';
            MID_OLED_ShowString(18, 28, pbuf, 12);
        }
        {
            int16_t ang = (int16_t)(s_servo_angle_f + 0.5f);
            MID_OLED_ShowString(72, 28, "SA:", 12);
            MID_OLED_ShowNumber(90, 28, (uint32_t)ang, 3, 12);
        }

        /* Line 40: distance */
        {
            char dbuf[5];
            app_ltc_fmt_dist(dbuf, s_distance_m);
            MID_OLED_ShowString(0, 40, "D:", 12);
            MID_OLED_ShowString(12, 40, dbuf, 12);
            MID_OLED_ShowString(54, 40, "/7.00m", 12);
        }

        /* Line 52: elapsed time + feedforward */
        {
            float elapsed = (float)(s_tick - s_start_tick) * 0.01f;
            char tbuf[7];
            uint16_t sec = (uint16_t)elapsed;
            uint16_t ds = (uint16_t)(elapsed * 10.0f) % 10;
            tbuf[0] = '0' + (sec / 10) % 10;
            tbuf[1] = '0' + sec % 10;
            tbuf[2] = '.';
            tbuf[3] = '0' + ds;
            tbuf[4] = 's';
            tbuf[5] = '\0';
            MID_OLED_ShowString(0, 52, tbuf, 12);
            {
                int32_t ff = (int32_t)(-KFF_ACCEL * s_car_accel);
                char ff_buf[5];
                uint8_t p2 = 0;
                ff_buf[p2++] = (ff < 0) ? '-' : '+';
                if (ff < 0) ff = -ff;
                ff_buf[p2++] = '0' + (ff / 10) % 10;
                ff_buf[p2++] = '0' + (ff % 10);
                ff_buf[p2++] = 'd';
                ff_buf[p2]   = '\0';
                MID_OLED_ShowString(48, 52, "FF:", 12);
                MID_OLED_ShowString(72, 52, ff_buf, 12);
            }
        }
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
    s_start_tick   = s_tick;

    /* Reset vision PID state for fresh run */
    s_i_accum        = 0.0f;
    s_ball_velocity  = 0.0f;
    s_vision_trim    = 0.0f;
    s_k230_pos       = 0.0f;
    s_k230_error     = 0.0f;
    s_k230_has_prev  = false;
    s_prev_k230_pos  = 0.0f;
    s_prev_k230_ts   = 0;
    s_prev_car_speed = 0.0f;
    s_car_accel      = 0.0f;

    s_running = true;
}

void APP_LineTrack_LowSpeedCircle_Stop(void)
{
    s_running = false;
    s_done    = false;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    s_servo_angle_f = 135.0f;
    s_i_accum       = 0.0f;
    BSP_Servo_SetAngle(135);
    BSP_Motor_Stop();
}

bool APP_LineTrack_LowSpeedCircle_IsRunning(void)
{
    return s_running;
}

#include "app_control.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"
#include "mid_line_track.h"
#include "mid_jy62.h"
#include "mid_gyro_hold.h"
#include "app_gyro_task.h"

/* ---- Physical constants ---- */
#define CONTROL_FREQ             100.0f
#define CONTROL_PERIMETER        0.2042f     /* wheel circumference: 65mm * pi */
#define CONTROL_PULSES_PER_REV   25525       /* calibrated encoder pulses per wheel rev */
#define CONTROL_SPEED_ALPHA      0.4f
#define CONTROL_DEADBAND         0.005f
#define CONTROL_PWM_MAX          7800
#define CONTROL_PID_KP           600.0f
#define CONTROL_PID_KI           450.0f

/* ---- Calibration constants ---- */
#define CONTROL_CALIB_SPEED   0.10f   /* slow speed for calibration (m/s) */
#define CONTROL_CALIB_PULSES  125000  /* encoder pulses for 1 meter */

/* ---- Motor speed state ---- */
typedef struct {
    float current_speed;    /* Current filtered speed (m/s) */
    float target_speed;     /* Target speed setpoint (m/s) */
    float pwm_output;       /* Last computed PWM value */
} motor_speed_t;

/* ---- Static state ---- */
static volatile motor_speed_t s_motor_left;
static volatile motor_speed_t s_motor_right;
static volatile bool s_flag_stop = true;
static volatile uint32_t s_display_tick = 0;
static volatile uint16_t s_sensor_data[BSP_GRAYSCALE_CHANNELS];

/* ---- Encoder calibration state ---- */
static volatile bool     s_calib_mode     = false;
static volatile uint32_t s_calib_pulses_a = 0;
static volatile uint32_t s_calib_pulses_b = 0;
static volatile bool     s_calib_done     = false;

/* ---- Direct drive mode (BLE !DRIVE) ---- */
static volatile bool   s_direct_mode  = false;
static volatile float  s_direct_speed = 0.0f;

/* ---- PI controller state (one set per motor) ---- */
static float s_last_bias_left;
static float s_last_bias_right;

/* ---- PWM output limit ---- */
static float control_pwm_limit(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

/* ---- PI controller: incremental discrete PI with deadband ---- */
static int16_t control_pi_update(float current, float target, float *last_bias,
    float *pwm)
{
    float bias = target - current;
    float abs_bias = (bias > 0.0f) ? bias : -bias;

    /* Deadband: stop PI accumulation when error is tiny */
    if (abs_bias < CONTROL_DEADBAND) {
        *last_bias = bias;
        return (int16_t)(*pwm);
    }

    /* Incremental PI: pwm += Kp * [e(k) - e(k-1)] + Ki * e(k) */
    *pwm += CONTROL_PID_KP * (bias - *last_bias) + CONTROL_PID_KI * bias;
    *last_bias = bias;
    *pwm = control_pwm_limit(*pwm, (float)(-CONTROL_PWM_MAX), (float)CONTROL_PWM_MAX);

    return (int16_t)(*pwm);
}

/* ---- Speed calculation: encoder counts to m/s with low-pass filter ---- */
static float control_calc_speed(int16_t encoder_count)
{
    /* raw_speed = counts * Freq * Perimeter / PulsesPerRev (calibrated) */
    float raw_speed = (float)encoder_count * CONTROL_FREQ * CONTROL_PERIMETER
        / (float)CONTROL_PULSES_PER_REV;

    return raw_speed;
}

static float control_lowpass_filter(float raw, float *filtered)
{
    *filtered = CONTROL_SPEED_ALPHA * raw + (1.0f - CONTROL_SPEED_ALPHA) * (*filtered);
    return *filtered;
}

/* ---- Public API ---- */

void APP_Control_Init(void)
{
    s_motor_left.current_speed  = 0.0f;
    s_motor_left.target_speed   = 0.0f;
    s_motor_left.pwm_output     = 0.0f;
    s_motor_right.current_speed = 0.0f;
    s_motor_right.target_speed  = 0.0f;
    s_motor_right.pwm_output    = 0.0f;
    s_flag_stop = true;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_display_tick = 0;
}

/*
 * Called from 10ms timer ISR callback.
 * Reads encoder → calculates speed → runs PI → sets PWM.
 * Order: LED → key → sensors → speed → calibration → target → PI.
 */
void APP_Control_TimerTick(void)
{
    int16_t count_a, count_b;
    float raw_a, raw_b;
    int16_t pwm_a, pwm_b;
    uint16_t raw_sensor[BSP_GRAYSCALE_CHANNELS];

    /* 0. LED: blink when running, solid on when stopped */
    if (s_flag_stop) {
        BSP_LED_Flash(0);
    } else {
        BSP_LED_Flash(100);
    }

    /* 1. Key scan (must come early for precise 10ms timing) */
    BSP_Key_Scan();

    /* 2. Read grayscale sensor */
    BSP_Grayscale_ReadAll(raw_sensor);

    /* 3. Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* 4. Convert to raw speed (m/s) */
    raw_a = control_calc_speed(count_a);
    raw_b = control_calc_speed(-count_b);  /* Motor B is mechanically reversed */

    /* 5. Low-pass filter */
    control_lowpass_filter(raw_a, &s_motor_left.current_speed);
    control_lowpass_filter(raw_b, &s_motor_right.current_speed);

    /* 6. Calibration mode: drive straight for 1 meter with gyro hold */
    if (s_calib_mode) {
        float correction;

        s_calib_pulses_a += (count_a > 0) ? (uint32_t)count_a : (uint32_t)(-count_a);
        s_calib_pulses_b += (count_b > 0) ? (uint32_t)count_b : (uint32_t)(-count_b);

        if (s_calib_pulses_a >= CONTROL_CALIB_PULSES ||
            s_calib_pulses_b >= CONTROL_CALIB_PULSES) {
            /* 1 meter complete: stop immediately */
            s_calib_done  = true;
            s_calib_mode  = false;
            s_flag_stop   = true;
            MID_GyroHold_Clear();
            BSP_Motor_Stop();
        } else {
            /* Drive straight with gyro hold correction */
            correction = MID_GyroHold_GetCorrection();
            s_motor_left.target_speed  = CONTROL_CALIB_SPEED + correction;
            s_motor_right.target_speed = CONTROL_CALIB_SPEED - correction;
            pwm_a = control_pi_update(s_motor_left.current_speed,
                s_motor_left.target_speed, &s_last_bias_left,
                &s_motor_left.pwm_output);
            pwm_b = control_pi_update(s_motor_right.current_speed,
                s_motor_right.target_speed, &s_last_bias_right,
                &s_motor_right.pwm_output);
            BSP_Motor_SetPWM(pwm_a, pwm_b);
        }
    }

    /* 7. Set target speeds based on mode (skip during calibration) */
    if (!s_calib_mode) {
        bool gyro_active = (APP_GyroTask_GetState() != APP_GYRO_TASK_IDLE);

        if (gyro_active || !s_flag_stop) {
            float left_tgt, right_tgt;

            if (gyro_active) {
                /* Gyro task mode: delegate speed targets to task planner */
                bool task_done = APP_GyroTask_Update(0.30f,
                    count_a, count_b, &left_tgt, &right_tgt);
                s_flag_stop = false;
                if (task_done) {
                    s_flag_stop = true;
                    BSP_Motor_Stop();
                }
            } else if (s_direct_mode) {
                /* Direct drive mode (!DRIVE): both wheels at same speed */
                left_tgt  = s_direct_speed;
                right_tgt = s_direct_speed;
            } else {
                /* Line-tracking mode: compute differential targets from sensor */
                MID_LineTrack_Update(raw_sensor,
                    &left_tgt, &right_tgt);
            }

            s_motor_left.target_speed  = left_tgt;
            s_motor_right.target_speed = right_tgt;
        }
    }

    /* Copy sensor data to volatile for display use */
    for (uint8_t i = 0; i < BSP_GRAYSCALE_CHANNELS; i++) {
        s_sensor_data[i] = raw_sensor[i];
    }

    /* 8. PI control and PWM output (skip when stopped or calibrating) */
    if (!s_flag_stop && !s_calib_mode) {
        pwm_a = control_pi_update(s_motor_left.current_speed,
            s_motor_left.target_speed, &s_last_bias_left,
            &s_motor_left.pwm_output);
        pwm_b = control_pi_update(s_motor_right.current_speed,
            s_motor_right.target_speed, &s_last_bias_right,
            &s_motor_right.pwm_output);
        BSP_Motor_SetPWM(pwm_a, pwm_b);
    }

    s_display_tick++;
}

/* Convert m/s to a 4-char signed string in cm/s, e.g. "+010", "-003" */
static void control_fmt_speed(char *buf, float speed_mps)
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

/*
 * Called from main loop. Updates OLED display every 500ms.
 * Layout (12px font, 128x64):
 *   L: +010|+009    (target | actual, cm/s)
 *   R: +010|+009
 *   Status: RUN/STOP
 */
void APP_Control_Run(void)
{
    char buf_speed[5];

    if (s_display_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    if (s_calib_done) {
        /* Calibration result screen */
        MID_OLED_ShowString(0, 0, "CALIB DONE", 12);
        MID_OLED_ShowString(0, 16, "A:", 12);
        MID_OLED_ShowNumber(18, 16, s_calib_pulses_a, 6, 12);
        MID_OLED_ShowString(0, 32, "B:", 12);
        MID_OLED_ShowNumber(18, 32, s_calib_pulses_b, 6, 12);
    } else if (s_calib_mode) {
        /* Calibration in progress */
        uint32_t pct = (s_calib_pulses_a * 100UL) / CONTROL_CALIB_PULSES;
        float yaw_err = MID_GyroHold_GetError();

        MID_OLED_ShowString(0, 0, "CALIB 1M", 12);
        MID_OLED_ShowNumber(72, 0, pct, 3, 12);
        MID_OLED_ShowString(90, 0, "%", 12);
        MID_OLED_ShowString(0, 16, "Y:", 12);
        if (yaw_err < 0) {
            MID_OLED_ShowString(12, 16, "-", 12);
            MID_OLED_ShowNumber(18, 16, (uint32_t)(-yaw_err), 3, 12);
        } else {
            MID_OLED_ShowString(12, 16, "+", 12);
            MID_OLED_ShowNumber(18, 16, (uint32_t)yaw_err, 3, 12);
        }
        MID_OLED_ShowString(0, 32, "A:", 12);
        MID_OLED_ShowNumber(18, 32, s_calib_pulses_a, 6, 12);
        MID_OLED_ShowString(0, 44, "B:", 12);
        MID_OLED_ShowNumber(18, 44, s_calib_pulses_b, 6, 12);
    } else if (s_flag_stop) {
        /* STOP screen */
        MID_OLED_ShowString(48, 24, "STOP", 12);
    } else if (APP_GyroTask_GetState() != APP_GYRO_TASK_IDLE) {
        /* Gyro task screen */
        app_gyro_task_state_t task_st = APP_GyroTask_GetState();
        const char *phase_str = "???";
        switch (task_st) {
        case APP_GYRO_TASK_TURN: phase_str = "TURN"; break;
        case APP_GYRO_TASK_DRIVE: phase_str = "DRIVE"; break;
        case APP_GYRO_TASK_DONE: phase_str = "DONE"; break;
        default: break;
        }
        MID_OLED_ShowString(0, 0, "Gyro:", 12);
        MID_OLED_ShowString(36, 0, phase_str, 12);

        control_fmt_speed(buf_speed, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf_speed, 12);

        control_fmt_speed(buf_speed, s_motor_right.current_speed);
        MID_OLED_ShowString(54, 16, "R:", 12);
        MID_OLED_ShowString(66, 16, buf_speed, 12);
    } else {
        uint8_t i, cnt = 0;
        for (i = 0; i < BSP_GRAYSCALE_CHANNELS; i++) {
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
            int error = MID_LineTrack_GetError();
            MID_OLED_ShowString(30, 0, "Er:", 12);
            if (error < 0) {
                MID_OLED_ShowString(54, 0, "-", 12);
                MID_OLED_ShowNumber(62, 0, (uint32_t)(-error), 2, 12);
            } else {
                MID_OLED_ShowNumber(54, 0, (uint32_t)error, 2, 12);
            }
        }

        /* Line 16: left + right actual speed */
        control_fmt_speed(buf_speed, s_motor_left.current_speed);
        MID_OLED_ShowString(0, 16, "L:", 12);
        MID_OLED_ShowString(12, 16, buf_speed, 12);

        control_fmt_speed(buf_speed, s_motor_right.current_speed);
        MID_OLED_ShowString(54, 16, "R:", 12);
        MID_OLED_ShowString(66, 16, buf_speed, 12);

        /* Line 32: status */
        MID_OLED_ShowString(0, 32, "RUN", 12);
    }

    MID_OLED_RefreshGram();
}

void APP_Control_ToggleStartStop(void)
{
    s_flag_stop = !s_flag_stop;

    if (!s_flag_stop) {
        /* Start tracking: reset line-track and PI state */
        MID_LineTrack_Reset();
        s_last_bias_left  = 0.0f;
        s_last_bias_right = 0.0f;
        s_motor_left.pwm_output  = 0.0f;
        s_motor_right.pwm_output = 0.0f;
    } else {
        /* Stop: zero targets and stop motors */
        s_motor_left.target_speed  = 0.0f;
        s_motor_right.target_speed = 0.0f;
        BSP_Motor_Stop();
    }
}

bool APP_Control_IsRunning(void)
{
    return (!s_flag_stop) || s_calib_done;
}

void APP_Control_StartCalibration(void)
{
    s_calib_mode     = true;
    s_calib_done     = false;
    s_calib_pulses_a = 0;
    s_calib_pulses_b = 0;
    s_flag_stop      = false;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;

    /* Lock current heading for straight-line gyro hold */
    MID_GyroHold_SetReference();
}

void APP_Control_StartDirect(float speed_mps)
{
    s_direct_mode  = true;
    s_direct_speed = speed_mps;
    s_flag_stop    = false;
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;
}

void APP_Control_StopDirect(void)
{
    s_direct_mode = false;
    s_flag_stop   = true;
    s_motor_left.target_speed  = 0.0f;
    s_motor_right.target_speed = 0.0f;
    BSP_Motor_Stop();
}

float APP_Control_GetSpeedA(void)
{
    return s_motor_left.current_speed;
}

float APP_Control_GetSpeedB(void)
{
    return s_motor_right.current_speed;
}

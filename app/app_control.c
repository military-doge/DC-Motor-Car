#include "app_control.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"
#include "mid_line_track.h"

/* ---- Physical constants ---- */
#define CONTROL_FREQ             100.0f
#define CONTROL_PERIMETER        0.2042f     /* wheel circumference: 65mm * pi */
#define CONTROL_PULSES_PER_REV   25525       /* calibrated encoder pulses per wheel rev */
#define CONTROL_SPEED_ALPHA      0.4f
#define CONTROL_DEADBAND         0.005f
#define CONTROL_PWM_MAX          7800

/* ---- PI runtime parameters (adjustable via BLE !PID) ---- */
static float s_pid_kp = 1225.0f;
static float s_pid_ki = 3600.0f;

/* ---- Sweep profile constants (BLE !SWP) ---- */
#define SWEEP_ACCEL_TICKS   100     /* 1s accelerate at 10ms tick */
#define SWEEP_HOLD_TICKS    200     /* 2s constant speed */
#define SWEEP_DECEL_TICKS   100     /* 1s decelerate */
#define SWEEP_TARGET_SPEED  0.30f   /* m/s */
#define SWEEP_SAMPLE_DIV    5       /* record every 5 ticks = 50ms */
#define SWEEP_MAX_SAMPLES   100
#define SWEEP_SCALE         1000    /* store speeds as mm/s (int16_t) */

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

/* ---- Direct drive mode (BLE !DRIVE) ---- */
static volatile bool   s_direct_mode  = false;
static volatile float  s_direct_speed = 0.0f;

/* ---- Sweep test state (BLE !SWP) ---- */
static volatile bool      s_sweep_active   = false;
static volatile bool      s_sweep_done     = false;
static uint16_t           s_sweep_tick;
static uint16_t           s_sweep_count;
static int16_t            s_sweep_target[SWEEP_MAX_SAMPLES];
static int16_t            s_sweep_actual_l[SWEEP_MAX_SAMPLES];
static int16_t            s_sweep_actual_r[SWEEP_MAX_SAMPLES];

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
    *pwm += s_pid_kp * (bias - *last_bias) + s_pid_ki * bias;
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
    s_pid_kp = 1225.0f;
    s_pid_ki = 3600.0f;
}

/*
 * Called from 10ms timer ISR callback.
 * Reads encoder → calculates speed → runs PI → sets PWM.
 * Order: LED → key → sensors → speed → target → PI.
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

    /* 6. Set target speeds based on mode */
    if (!s_flag_stop) {
        float left_tgt, right_tgt;

        if (s_sweep_active) {
                /* Sweep test mode (!SWP): trapezoidal speed profile */
                uint16_t total_ticks = SWEEP_ACCEL_TICKS + SWEEP_HOLD_TICKS + SWEEP_DECEL_TICKS;
                float target;

                if (s_sweep_tick < SWEEP_ACCEL_TICKS) {
                    /* Accelerating: 0 → target speed */
                    target = SWEEP_TARGET_SPEED * (float)s_sweep_tick / (float)SWEEP_ACCEL_TICKS;
                } else if (s_sweep_tick < SWEEP_ACCEL_TICKS + SWEEP_HOLD_TICKS) {
                    /* Holding at target speed */
                    target = SWEEP_TARGET_SPEED;
                } else if (s_sweep_tick < total_ticks) {
                    /* Decelerating: target speed → 0 */
                    uint16_t decel_tick = s_sweep_tick - SWEEP_ACCEL_TICKS - SWEEP_HOLD_TICKS;
                    target = SWEEP_TARGET_SPEED * (float)(SWEEP_DECEL_TICKS - decel_tick) / (float)SWEEP_DECEL_TICKS;
                } else {
                    /* Profile complete */
                    target = 0.0f;
                    s_sweep_active = false;
                    s_sweep_done   = true;
                    s_flag_stop    = true;
                    BSP_Motor_Stop();
                }

                left_tgt  = target;
                right_tgt = target;

                /* Record sample */
                if (s_sweep_active && (s_sweep_tick % SWEEP_SAMPLE_DIV == 0)
                    && s_sweep_count < SWEEP_MAX_SAMPLES) {
                    s_sweep_target[s_sweep_count]  = (int16_t)(target * SWEEP_SCALE);
                    s_sweep_actual_l[s_sweep_count] = (int16_t)(s_motor_left.current_speed * SWEEP_SCALE);
                    s_sweep_actual_r[s_sweep_count] = (int16_t)(s_motor_right.current_speed * SWEEP_SCALE);
                    s_sweep_count++;
                }
                s_sweep_tick++;
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

    /* Copy sensor data to volatile for display use */
    for (uint8_t i = 0; i < BSP_GRAYSCALE_CHANNELS; i++) {
        s_sensor_data[i] = raw_sensor[i];
    }

    /* 7. PI control and PWM output (skip when stopped) */
    if (!s_flag_stop) {
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

    if (s_flag_stop) {
        /* STOP screen */
        MID_OLED_ShowString(48, 24, "STOP", 12);
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
    return !s_flag_stop;
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

void APP_Control_StartSweep(float kp, float ki)
{
    /* Set PI parameters for the sweep */
    APP_Control_SetPID(kp, ki);

    /* Reset sweep state */
    s_sweep_active = false;
    s_sweep_done   = false;
    s_sweep_tick   = 0;
    s_sweep_count  = 0;

    /* Reset PI state for clean start */
    s_last_bias_left  = 0.0f;
    s_last_bias_right = 0.0f;
    s_motor_left.pwm_output  = 0.0f;
    s_motor_right.pwm_output = 0.0f;

    s_flag_stop    = false;
    s_sweep_active = true;
}

bool APP_Control_IsSweepDone(void)
{
    bool done = s_sweep_done;
    s_sweep_done = false;  /* Clear after read */
    return done;
}

uint16_t APP_Control_GetSweepCount(void)
{
    return s_sweep_count;
}

int16_t *APP_Control_GetSweepTarget(void)
{
    return s_sweep_target;
}

int16_t *APP_Control_GetSweepActualL(void)
{
    return s_sweep_actual_l;
}

int16_t *APP_Control_GetSweepActualR(void)
{
    return s_sweep_actual_r;
}

void APP_Control_SetPID(float kp, float ki)
{
    if (kp < 0.0f)  kp = 0.0f;
    if (kp > 5000.0f) kp = 5000.0f;
    if (ki < 0.0f)  ki = 0.0f;
    if (ki > 5000.0f) ki = 5000.0f;
    s_pid_kp = kp;
    s_pid_ki = ki;
}

void APP_Control_GetPID(float *kp, float *ki)
{
    *kp = s_pid_kp;
    *ki = s_pid_ki;
}

float APP_Control_GetSpeedA(void)
{
    return s_motor_left.current_speed;
}

float APP_Control_GetSpeedB(void)
{
    return s_motor_right.current_speed;
}

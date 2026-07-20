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
#define CONTROL_PERIMETER        0.2104867f
#define CONTROL_ENCODER_LINES    13
#define CONTROL_MULTIPLY_FACTOR  2
#define CONTROL_GEAR_RATIO       20
#define CONTROL_SPEED_ALPHA      0.4f
#define CONTROL_DEADBAND         0.005f
#define CONTROL_PWM_MAX          7800
#define CONTROL_PID_KP           600.0f
#define CONTROL_PID_KI           450.0f

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
    /* raw_speed = counts * Freq * Perimeter / (LINES * MULTIPLY * GEAR_RATIO) */
    float raw_speed = (float)encoder_count * CONTROL_FREQ * CONTROL_PERIMETER
        / ((float)CONTROL_ENCODER_LINES * CONTROL_MULTIPLY_FACTOR
           * CONTROL_GEAR_RATIO);

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
 * Also handles key scan (needs 10ms timing) and LED blink.
 */
void APP_Control_TimerTick(void)
{
    int16_t count_a, count_b;
    float raw_a, raw_b;
    int16_t pwm_a, pwm_b;
    uint16_t raw_sensor[BSP_GRAYSCALE_CHANNELS];

    /* 0. Read grayscale sensor (always, for display) */
    BSP_Grayscale_ReadAll(raw_sensor);

    /* 1. Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* 2. Convert to raw speed (m/s) */
    raw_a = control_calc_speed(count_a);
    raw_b = control_calc_speed(-count_b);  /* Motor B is mechanically reversed */

    /* 3. Low-pass filter */
    control_lowpass_filter(raw_a, &s_motor_left.current_speed);
    control_lowpass_filter(raw_b, &s_motor_right.current_speed);

    /* 4. Set target speeds based on mode */
    if (!s_flag_stop) {
        float left_tgt, right_tgt;
        /* Tracking mode: compute differential targets from line sensor */
        MID_LineTrack_Update(raw_sensor,
            &left_tgt, &right_tgt);
        s_motor_left.target_speed  = left_tgt;
        s_motor_right.target_speed = right_tgt;
    }

    /* Copy sensor data to volatile for display use */
    for (uint8_t i = 0; i < BSP_GRAYSCALE_CHANNELS; i++) {
        s_sensor_data[i] = raw_sensor[i];
    }

    /* 5. PI control and PWM output (skip when stopped) */
    if (!s_flag_stop) {
        pwm_a = control_pi_update(s_motor_left.current_speed,
            s_motor_left.target_speed, &s_last_bias_left,
            &s_motor_left.pwm_output);
        pwm_b = control_pi_update(s_motor_right.current_speed,
            s_motor_right.target_speed, &s_last_bias_right,
            &s_motor_right.pwm_output);
        BSP_Motor_SetPWM(pwm_a, pwm_b);
    }

    /* 6. LED blink (100 ticks = 1 second period) */
    BSP_LED_Flash(100);

    /* 7. Key scan (needs 10ms precise timing) */
    BSP_Key_Scan();

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

float APP_Control_GetSpeedA(void)
{
    return s_motor_left.current_speed;
}

float APP_Control_GetSpeedB(void)
{
    return s_motor_right.current_speed;
}

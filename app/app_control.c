#include "app_control.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "mid_oled.h"

/* ---- Physical constants ---- */
#define CONTROL_FREQ             100.0f
#define CONTROL_PERIMETER        0.2104867f
#define CONTROL_ENCODER_LINES    13
#define CONTROL_MULTIPLY_FACTOR  2
#define CONTROL_GEAR_RATIO       30
#define CONTROL_SPEED_ALPHA      0.4f
#define CONTROL_DEADBAND         0.005f
#define CONTROL_PWM_MAX          7800
#define CONTROL_PID_KP           400.0f
#define CONTROL_PID_KI           300.0f
#define CONTROL_TARGET_SPEED     0.10f

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

    /* 4. PI control and PWM output (skip when stopped) */
    if (!s_flag_stop) {
        pwm_a = control_pi_update(s_motor_left.current_speed,
            s_motor_left.target_speed, &s_last_bias_left,
            &s_motor_left.pwm_output);
        pwm_b = control_pi_update(s_motor_right.current_speed,
            s_motor_right.target_speed, &s_last_bias_right,
            &s_motor_right.pwm_output);
        BSP_Motor_SetPWM(pwm_a, pwm_b);
    }

    /* 5. LED blink (100 ticks = 1 second period) */
    BSP_LED_Flash(100);

    /* 6. Key scan (needs 10ms precise timing) */
    BSP_Key_Scan();

    s_display_tick++;
}

/*
 * Called from main loop. Updates OLED display every 500ms.
 */
void APP_Control_Run(void)
{
    if (s_display_tick % 50 != 0) {
        return;
    }

    /* Display left motor speed in cm/s */
    MID_OLED_ShowString(0, 0, "MA_V:", 12);
    MID_OLED_ShowNumber(40, 0,
        (uint32_t)(s_motor_left.current_speed * 100.0f), 4, 12);

    /* Display right motor speed in cm/s */
    MID_OLED_ShowString(0, 20, "MB_V:", 12);
    MID_OLED_ShowNumber(40, 20,
        (uint32_t)(s_motor_right.current_speed * 100.0f), 4, 12);

    /* Display run/stop status */
    MID_OLED_ShowString(0, 40, "Status:", 12);
    MID_OLED_ShowString(60, 40, s_flag_stop ? "STOP" : "RUN ", 12);

    MID_OLED_RefreshGram();
}

void APP_Control_ToggleStartStop(void)
{
    s_flag_stop = !s_flag_stop;

    if (!s_flag_stop) {
        /* Start: set target speeds for both motors */
        s_motor_left.target_speed  = CONTROL_TARGET_SPEED;
        s_motor_right.target_speed = CONTROL_TARGET_SPEED;
        /* Reset PI state for clean start */
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

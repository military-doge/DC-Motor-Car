#include "app_test.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_servo.h"
#include "bsp_led.h"
#include "bsp_delay.h"
#include "mid_oled.h"

/*
 * Encoder calibration: 1m distance test.
 *
 * Key press → car drives straight 1m (pulse count) → stops.
 * OLED shows total pulse count for user to measure actual distance.
 *
 * Calibration formula:
 *   new_pulses_per_rev = 25525 * actual_cm / 100
 */

/* ---- Calibration constants ---- */
#define TEST_PERIMETER           0.2042f
#define TEST_PULSES_PER_REV      25525
#define TEST_DIST_PER_PULSE      (TEST_PERIMETER / TEST_PULSES_PER_REV / 2.0f)
#define TEST_TARGET_M            1.0f
#define TEST_TARGET_PULSES       ((int32_t)(TEST_TARGET_M / TEST_DIST_PER_PULSE))

/* Speed: slow constant for precision */
#define TEST_SPEED_MPS           0.12f
#define TEST_PWM_MAX             7800
#define TEST_PWM_DEADBAND        200

/* ---- Static state ---- */
static bool                s_running  = false;
static bool                s_done     = false;
static volatile int32_t    s_total_pulses;
static volatile float      s_distance_m;
static volatile int16_t    s_pwm_output;

/* ========== public API ========== */

void APP_Test_Init(void)
{
    s_running       = false;
    s_done          = false;
    s_total_pulses  = 0;
    s_distance_m    = 0.0f;
    s_pwm_output    = 0;

    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_Servo_Init();
    BSP_Servo_SetAngle(135);

    MID_OLED_Clear();
    MID_OLED_ShowString(0, 0, "ENCODER CAL 1m", 12);
    MID_OLED_ShowString(0, 24, "Target: 1.00m", 12);
    MID_OLED_ShowString(0, 40, "Key=Start", 12);
    MID_OLED_RefreshGram();
}

void APP_Test_TimerTick(void)
{
    int16_t count_a, count_b;
    int16_t pwm;

    if (!s_running) return;

    /* Read and reset encoder counts */
    count_a = BSP_Encoder_GetCountA();
    count_b = BSP_Encoder_GetCountB();
    BSP_Encoder_ResetCounts();

    /* Odometry */
    s_total_pulses += (count_a > 0 ? count_a : -count_a)
                    + (count_b > 0 ? count_b : -count_b);
    s_distance_m = (float)s_total_pulses * TEST_DIST_PER_PULSE;

    /* Auto-stop at target */
    if (s_total_pulses >= TEST_TARGET_PULSES) {
        s_running = false;
        s_done    = true;
        BSP_Motor_Brake();
        s_pwm_output = 0;
        return;
    }

    /* Open-loop straight drive at constant PWM */
    /* Simple ramp up to target speed */
    {
        int16_t target_pwm = (int16_t)(TEST_SPEED_MPS / 0.35f * TEST_PWM_MAX);
        if (target_pwm > 3000) target_pwm = 3000;
        if (target_pwm < TEST_PWM_DEADBAND) target_pwm = TEST_PWM_DEADBAND;

        if (s_pwm_output < target_pwm - 100) {
            s_pwm_output += 100;
        } else if (s_pwm_output > target_pwm + 100) {
            s_pwm_output -= 100;
        } else {
            s_pwm_output = target_pwm;
        }
    }

    pwm = s_pwm_output;
    BSP_Motor_SetPWM(pwm, pwm);
}

void APP_Test_Run(void)
{
    char buf[14];
    uint8_t p;

    MID_OLED_Clear();

    if (!s_done && !s_running) {
        /* ---- IDLE ---- */
        MID_OLED_ShowString(0, 0, "ENCODER CAL 1m", 12);
        MID_OLED_ShowString(0, 24, "Target: 1.00m", 12);
        MID_OLED_ShowString(0, 40, "Key=Start", 12);
        MID_OLED_ShowString(0, 52, "Pulses/rev=25525", 12);
    } else if (s_done) {
        /* ---- DONE ---- */
        MID_OLED_ShowString(0, 0, "TEST  DONE", 12);

        /* Total pulses */
        MID_OLED_ShowString(0, 16, "P:", 12);
        {
            int32_t v = s_total_pulses;
            p = 0;
            if (v < 0) { buf[p++] = '-'; v = -v; }
            if (v >= 1000000) { buf[p++] = '0' + v / 1000000; v %= 1000000; }
            if (v >= 100000)  { buf[p++] = '0' + v / 100000;  v %= 100000;  }
            buf[p++] = '0' + v / 10000; v %= 10000;
            buf[p++] = '0' + v / 1000;  v %= 1000;
            buf[p++] = '0' + v / 100;   v %= 100;
            buf[p++] = '0' + v / 10;    v %= 10;
            buf[p++] = '0' + v;
            buf[p] = '\0';
            MID_OLED_ShowString(18, 16, buf, 12);
        }

        /* Measured distance (from encoder) */
        {
            int32_t mm = (int32_t)(s_distance_m * 1000.0f + 0.5f);
            MID_OLED_ShowString(0, 28, "Enc:", 12);
            p = 0;
            if (mm >= 1000) { buf[p++] = '0' + mm / 1000; mm %= 1000; }
            buf[p++] = '0' + mm / 100; mm %= 100;
            buf[p++] = '.';
            buf[p++] = '0' + mm / 10; mm %= 10;
            buf[p++] = '0' + mm;
            buf[p++] = 'm'; buf[p] = '\0';
            MID_OLED_ShowString(30, 28, buf, 12);
        }

        /* Hint: measure actual distance */
        MID_OLED_ShowString(0, 44, "Measure actual:", 12);
        MID_OLED_ShowString(0, 56, "new=25525*100/act", 12);
    } else {
        /* ---- RUNNING ---- */
        MID_OLED_ShowString(0, 0, "RUNNING...", 12);

        /* Current distance */
        {
            int32_t mm = (int32_t)(s_distance_m * 1000.0f + 0.5f);
            MID_OLED_ShowString(0, 16, "Dist:", 12);
            p = 0;
            if (mm >= 1000) { buf[p++] = '0' + mm / 1000; mm %= 1000; }
            buf[p++] = '0' + mm / 100; mm %= 100;
            buf[p++] = '.';
            buf[p++] = '0' + mm / 10; mm %= 10;
            buf[p++] = '0' + mm;
            buf[p++] = 'm'; buf[p] = '\0';
            MID_OLED_ShowString(42, 16, buf, 12);
        }

        /* Pulse count */
        {
            int32_t v = s_total_pulses;
            MID_OLED_ShowString(0, 32, "Pulses:", 12);
            p = 0;
            v = (v >= 0 ? v : -v);
            buf[p++] = '0' + v / 100000 % 10;
            buf[p++] = '0' + v / 10000  % 10;
            buf[p++] = '0' + v / 1000   % 10;
            buf[p++] = '0' + v / 100    % 10;
            buf[p++] = '0' + v / 10     % 10;
            buf[p++] = '0' + v % 10;
            buf[p] = '\0';
            MID_OLED_ShowString(54, 32, buf, 12);
        }

        /* PWM */
        {
            int16_t v = s_pwm_output;
            MID_OLED_ShowString(0, 48, "PWM:", 12);
            p = 0;
            if (v < 0) { buf[p++] = '-'; v = -v; }
            buf[p++] = '0' + v / 1000 % 10;
            buf[p++] = '0' + v / 100  % 10;
            buf[p++] = '0' + v / 10   % 10;
            buf[p++] = '0' + v % 10;
            buf[p] = '\0';
            MID_OLED_ShowString(36, 48, buf, 12);
        }
    }

    MID_OLED_RefreshGram();
}

void APP_Test_Start(void)
{
    s_total_pulses  = 0;
    s_distance_m    = 0.0f;
    s_pwm_output    = 0;
    s_done          = false;
    s_running       = true;

    BSP_Encoder_ResetCounts();
    BSP_Motor_Init();
}

void APP_Test_Stop(void)
{
    s_running = false;
    s_done    = false;
    BSP_Motor_Brake();
    s_pwm_output = 0;
}

bool APP_Test_IsRunning(void)
{
    return s_running || s_done;
}

#include "app_test.h"
#include "bsp_servo.h"
#include "mid_oled.h"
#include "bsp_led.h"

/*
 * Servo → rack calibration test.
 *
 * Each key press advances one step through a symmetric angle sequence:
 *   +2° → +8° → +15° → -15° → -8° → -2°  (repeat)
 *
 * Angles are relative (delta from current position), cumulative.
 * Sequence total = 0°, so it always returns to centre after a full cycle.
 *
 * OLED shows:
 *   Step N     direction
 *   Servo: XXX deg
 *   Rack h: ±X.XXX mm  (theoretical)
 *
 * User measures actual rack height with caliper at each step
 * and compares against the theoretical value.
 */

/* ========== test sequence ========== */

static const int16_t s_seq_deltas[] = {2, 8, 15, -15, -8, -2};
#define SEQ_LEN  (sizeof(s_seq_deltas) / sizeof(s_seq_deltas[0]))

/* ========== physical constants ========== */

#define GEAR_RADIUS_MM   15.625f   /* pinion pitch-circle radius */
#define SERVO_CENTER     135

/* rack height from centre: h = (135 - θ) × r × π/180 */
static float calc_rack_h_mm(int16_t angle_deg)
{
    return (float)(SERVO_CENTER - angle_deg) * GEAR_RADIUS_MM * 3.14159265f / 180.0f;
}

/* ========== static state ========== */

static uint8_t  s_seq_index  = 0;
static int16_t  s_angle      = SERVO_CENTER;   /* absolute servo angle */
static bool     s_dirty      = true;           /* OLED needs refresh */
static bool     s_running    = false;          /* test active */

/* ========== public API ========== */

void APP_Test_Init(void)
{
    s_seq_index = 0;
    s_angle     = SERVO_CENTER;
    s_dirty     = true;
    s_running   = true;
    BSP_Servo_SetAngle(SERVO_CENTER);

    /* Show initial screen */
    MID_OLED_Clear();
    MID_OLED_ShowString(30, 0, "SERVO CAL", 12);
    MID_OLED_ShowString(24, 20, "Key=Step", 12);
    MID_OLED_ShowString(12, 36, "Measure rack", 12);
    MID_OLED_ShowString(12, 48, "with caliper", 12);
    MID_OLED_RefreshGram();
}

void APP_Test_TimerTick(void)
{
    /* nothing to do — all work in Start() triggered by key */
}

void APP_Test_Run(void)
{
    char buf[16];

    if (!s_dirty) return;
    s_dirty = false;

    MID_OLED_Clear();

    /* Line 0: step info */
    {
        int16_t delta = s_seq_deltas[s_seq_index > 0 ? s_seq_index - 1 : SEQ_LEN - 1];
        if (delta > 0) {
            MID_OLED_ShowString(0, 0, "Step", 12);
            MID_OLED_ShowNumber(36, 0, s_seq_index, 1, 12);
            MID_OLED_ShowString(48, 0, "+", 12);
            MID_OLED_ShowNumber(54, 0, delta, 2, 12);
            MID_OLED_ShowString(78, 0, "deg", 12);
        } else {
            MID_OLED_ShowString(0, 0, "Step", 12);
            MID_OLED_ShowNumber(36, 0, s_seq_index, 1, 12);
            MID_OLED_ShowString(48, 0, "-", 12);
            MID_OLED_ShowNumber(54, 0, -delta, 2, 12);
            MID_OLED_ShowString(78, 0, "deg", 12);
        }
    }

    /* Line 16: absolute servo angle */
    MID_OLED_ShowString(0, 16, "Servo:", 12);
    MID_OLED_ShowNumber(48, 16, (uint16_t)s_angle, 3, 12);
    MID_OLED_ShowString(78, 16, "deg", 12);

    /* Line 32: theoretical rack height */
    {
        float h = calc_rack_h_mm(s_angle);
        int32_t h_int = (int32_t)(h * 1000.0f);  /* μm */
        uint8_t pos = 0;
        MID_OLED_ShowString(0, 32, "h_th:", 12);
        if (h_int < 0) {
            buf[pos++] = '-';
            h_int = -h_int;
        } else {
            buf[pos++] = '+';
        }
        buf[pos++] = '0' + (h_int / 1000) % 10;
        buf[pos++] = '.';
        buf[pos++] = '0' + (h_int / 100) % 10;
        buf[pos++] = '0' + (h_int / 10) % 10;
        buf[pos++] = '0' + h_int % 10;
        buf[pos]   = '\0';
        buf[6]    = '\0';  /* truncate to "±X.XXX" */
        MID_OLED_ShowString(42, 32, buf, 12);
        MID_OLED_ShowString(90, 32, "mm", 12);
    }

    /* Line 48: hint */
    MID_OLED_ShowString(0, 52, "Key=Next", 12);

    MID_OLED_RefreshGram();
}

void APP_Test_Start(void)
{
    /* Advance to next step in sequence */
    int16_t delta = s_seq_deltas[s_seq_index];
    s_angle += delta;

    /* Clamp to servo range */
    if (s_angle < 0)   s_angle = 0;
    if (s_angle > 270) s_angle = 270;

    BSP_Servo_SetAngle((uint16_t)s_angle);
    s_seq_index = (s_seq_index + 1) % SEQ_LEN;
    s_dirty = true;
}

void APP_Test_Stop(void)
{
    s_running   = false;
    s_angle     = SERVO_CENTER;
    s_seq_index = 0;
    s_dirty     = true;
    BSP_Servo_SetAngle(SERVO_CENTER);
}

bool APP_Test_IsRunning(void)
{
    return s_running;
}

#include "app_test.h"
#include "bsp_servo.h"
#include "mid_k230.h"
#include "mid_oled.h"
#include "bsp_delay.h"

/*
 * K230 real-time data monitor.
 *
 * Continuously reads MID_K230 parsed frames and displays:
 *   - Ball position (cm, 0.1 cm precision)
 *   - Detection status (D flag)
 *   - Data age (ms since last valid frame)
 *
 * Key press toggles display on/off (same as other app modules).
 */

/* ========== static state ========== */

static bool s_running = false;

/* ========== public API ========== */

void APP_Test_Init(void)
{
    s_running = true;

    /* Center servo to 135° (horizontal) */
    BSP_Servo_Init();
    BSP_Servo_SetAngle(135);

    MID_OLED_Clear();
    MID_OLED_ShowString(24, 0, "K230 TEST", 12);
    MID_OLED_ShowString(6, 24, "Servo centered", 12);
    MID_OLED_ShowString(6, 40, "K230 data...", 12);
    MID_OLED_RefreshGram();
}

void APP_Test_TimerTick(void)
{
    /* nothing — K230 polling is in main loop, display in Run() */
}

void APP_Test_Run(void)
{
    char buf[14];

    if (!s_running) return;

    bool     detected = MID_K230_IsDetected();
    float    pos      = MID_K230_GetPosition();
    uint32_t last_up  = MID_K230_GetLastUpdate();

    MID_OLED_Clear();

    /* ---- Row 0: title ---- */
    MID_OLED_ShowString(24, 0, "K230  MONITOR", 12);

    /* ---- Row 1 (y=16): ball position ---- */
    if (detected) {
        /*
         * pos is in mm (K230 sends mm via cx_to_mm).
         * e.g. +35.0 → 35 mm, -120.0 → -120 mm
         */
        int32_t val = (int32_t)(pos >= 0 ? pos + 0.5f : pos - 0.5f);
        uint8_t p   = 0;

        MID_OLED_ShowString(0, 16, "X:", 12);

        if (val < 0) { buf[p++] = '-'; val = -val; }
        else         { buf[p++] = '+'; }

        buf[p++] = '0' + (val / 100) % 10;   /* hundreds mm */
        buf[p++] = '0' + (val / 10) % 10;    /* tens mm */
        buf[p++] = '0' + (val % 10);          /* ones mm */
        buf[p++] = 'm';
        buf[p++] = 'm';
        buf[p]   = '\0';

        MID_OLED_ShowString(18, 16, buf, 12);
    } else {
        MID_OLED_ShowString(0, 16, "X:  -- mm", 12);
    }

    /* ---- Row 2 (y=32): detection status ---- */
    MID_OLED_ShowString(0, 32, "D:", 12);
    MID_OLED_ShowNumber(18, 32, detected ? 1 : 0, 1, 12);
    MID_OLED_ShowString(30, 32, detected ? "BALL OK" : "NO BALL", 12);

    /* ---- Row 3 (y=48): data age ---- */
    MID_OLED_ShowString(0, 48, "Age:", 12);
    if (last_up > 0) {
        uint32_t age = BSP_Delay_GetTick() - last_up;
        if (age > 9999) age = 9999;             /* clamp to 4-digit display */
        MID_OLED_ShowNumber(36, 48, age, 4, 12);
        MID_OLED_ShowString(66, 48, "ms", 12);
    } else {
        MID_OLED_ShowString(36, 48, "NO DATA", 12);
    }

    MID_OLED_RefreshGram();
}

void APP_Test_Start(void)
{
    s_running = true;
}

void APP_Test_Stop(void)
{
    s_running = false;
}

bool APP_Test_IsRunning(void)
{
    return s_running;
}

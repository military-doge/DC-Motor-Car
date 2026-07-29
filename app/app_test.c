#include "app_test.h"
#include "bsp_servo.h"
#include "bsp_led.h"
#include "mid_oled.h"

static bool s_running = false;
static uint16_t s_angle = 0;
static int8_t s_direction = 1;
static uint16_t s_tick = 0;

void APP_Test_Init(void)
{
    s_running = false;
    s_angle = 135;
    s_direction = 1;
    s_tick = 0;
}

void APP_Test_TimerTick(void)
{
    s_tick++;

    /* Heartbeat: toggle LED every 50 ticks (500ms) to prove ISR is alive */
    if ((s_tick % 50) == 0) {
        BSP_LED_Toggle();
    }

    if (!s_running) return;

    s_angle += s_direction;

    if (s_angle >= 270) {
        s_angle = 270;
        s_direction = -1;
    } else if (s_angle == 0) {
        s_direction = 1;
    }

    BSP_Servo_SetAngle(s_angle);
}

void APP_Test_Run(void)
{
    static uint16_t last_tick = 0;
    uint16_t pulse;

    /* Refresh OLED every 200ms (20 ticks) */
    if (s_tick - last_tick < 20 && last_tick != 0) return;
    last_tick = s_tick;

    MID_OLED_Clear();

    MID_OLED_ShowString(0, 0, s_running ? "SRV ON " : "SRV OFF", 12);

    MID_OLED_ShowString(0, 16, "A:", 12);
    MID_OLED_ShowNumber(16, 16, s_angle, 3, 12);

    pulse = 500 + (uint32_t)s_angle * 2000 / 270;
    MID_OLED_ShowString(0, 32, "P:", 12);
    MID_OLED_ShowNumber(16, 32, pulse, 5, 12);

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

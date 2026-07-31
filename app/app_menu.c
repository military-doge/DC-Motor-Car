/*
 * App Menu Switcher.
 *
 * Double-click: cycle to next app
 * Single-click: start/stop current app
 *
 * OLED layout (overlay):
 *   y=0  app name + ON/OFF      (menu layer)
 *   y=12 timer MM:SS.CC          (menu layer)
 *   y=24+ app content            (app draws, overlay-corrected)
 */

#include "app_menu.h"

#include "app_line_track_high_speed.h"
#include "app_ball_ctrl_1.h"
#include "app_line_track_low_speed_straight.h"
#include "app_line_track_low_speed_circle.h"
#include "app_line_track_q6_hold_pos.h"
#include "app_test.h"
#include "app_control.h"

#include "bsp_key.h"
#include "bsp_servo.h"
#include "bsp_motor.h"
#include "bsp_dma_rx.h"
#include "mid_oled.h"

/* ========== App descriptor ========== */

typedef struct {
    const char *name;
    void (*init)(void);
    void (*tick)(void);
    void (*run)(void);
    void (*start)(void);
    void (*stop)(void);
    bool (*is_running)(void);
} app_entry_t;

/* ========== App table (order = double-click cycle) ========== */

static const app_entry_t g_apps[] = {
    { "Q2 LINE-TRACK",   APP_LineTrack_Init,                APP_LineTrack_TimerTick,                APP_LineTrack_Run,                APP_LineTrack_Start,                APP_LineTrack_Stop,                APP_LineTrack_IsRunning                },
    { "Q3 BALANCE",      APP_BallCtrl1_Init,                APP_BallCtrl1_TimerTick,                APP_BallCtrl1_Run,                APP_BallCtrl1_Start,                APP_BallCtrl1_Stop,                APP_BallCtrl1_IsRunning                },
    { "Q4 A->B",         APP_LineTrack_LowSpeedStraight_Init,  APP_LineTrack_LowSpeedStraight_TimerTick,  APP_LineTrack_LowSpeedStraight_Run,  APP_LineTrack_LowSpeedStraight_Start,  APP_LineTrack_LowSpeedStraight_Stop,  APP_LineTrack_LowSpeedStraight_IsRunning  },
    { "Q5 CIRCLE",       APP_LineTrack_LowSpeedCircle_Init, APP_LineTrack_LowSpeedCircle_TimerTick, APP_LineTrack_LowSpeedCircle_Run, APP_LineTrack_LowSpeedCircle_Start, APP_LineTrack_LowSpeedCircle_Stop, APP_LineTrack_LowSpeedCircle_IsRunning },
    { "Q6 HOLD POS",     APP_Q6_HoldPos_Init,              APP_Q6_HoldPos_TimerTick,              APP_Q6_HoldPos_Run,              APP_Q6_HoldPos_Start,              APP_Q6_HoldPos_Stop,              APP_Q6_HoldPos_IsRunning              },
    { "TEST ENC",        APP_Test_Init,                    APP_Test_TimerTick,                    APP_Test_Run,                    APP_Test_Start,                    APP_Test_Stop,                    APP_Test_IsRunning                    },
    { "CONTROL",         APP_Control_Init,                 APP_Control_TimerTick,                 APP_Control_Run,                 APP_Control_Start,                 APP_Control_Stop,                 APP_Control_IsRunning                 },
};

#define APP_COUNT  (sizeof(g_apps) / sizeof(g_apps[0]))

/* ========== Menu state ========== */

static uint8_t  g_current        = 0;
static uint32_t g_tick           = 0;
static uint32_t g_elapsed_ticks  = 0;   /* centiseconds since last start */

/* ========== Key handlers ========== */

static void on_single_click(void)
{
    if (g_apps[g_current].is_running()) {
        g_apps[g_current].stop();
    } else {
        g_apps[g_current].start();
    }
    g_elapsed_ticks = 0;  /* reset timer on every start/stop */
}

static void on_double_click(void)
{
    /* Stop current app */
    if (g_apps[g_current].is_running()) {
        g_apps[g_current].stop();
    }
    /* Safety: center servo, stop motors */
    BSP_Servo_SetAngle(135);
    BSP_Motor_Stop();
    g_elapsed_ticks = 0;

    /* Cycle to next app */
    g_current = (g_current + 1) % APP_COUNT;
    g_apps[g_current].init();
}

/* ========== OLED header ========== */

static void draw_header(void)
{
    char buf[13];

    /* Row 0 (y=0): app name (left) + ON/OFF (right) */
    MID_OLED_ShowString(0, 0, g_apps[g_current].name, 12);
    MID_OLED_ShowString(96, 0,
        g_apps[g_current].is_running() ? " ON" : "OFF", 12);

    /* Row 1 (y=12): timer MM:SS.CC */
    {
        uint32_t cs  = g_elapsed_ticks;
        uint32_t min = cs / 6000;
        uint32_t sec = (cs / 100) % 60;
        uint32_t frac = cs % 100;

        buf[0] = '0' + (min / 10) % 10;
        buf[1] = '0' + (min % 10);
        buf[2] = ':';
        buf[3] = '0' + (sec / 10) % 10;
        buf[4] = '0' + (sec % 10);
        buf[5] = '.';
        buf[6] = '0' + (frac / 10) % 10;
        buf[7] = '0' + (frac % 10);
        buf[8] = '\0';
        MID_OLED_ShowString(0, 12, buf, 12);
    }
}

/* ========== Public API ========== */

void APP_Menu_Init(void)
{
    g_current       = 0;
    g_tick          = 0;
    g_elapsed_ticks = 0;

    /* Init first app */
    g_apps[0].init();

    /* Register key handlers */
    BSP_Key_RegisterClickCallback(on_single_click);
    BSP_Key_RegisterDoubleClickCallback(on_double_click);
}

void APP_Menu_TimerTick(void)
{
    BSP_Key_Scan();
    BSP_DMA_RX_Process();

    g_tick++;

    /* Advance timer only when current app is running */
    if (g_apps[g_current].is_running()) {
        g_elapsed_ticks++;
    }

    /* Dispatch to current app */
    g_apps[g_current].tick();
}

void APP_Menu_Run(void)
{
    /* 2Hz OLED update (every 500ms = 50 ticks) */
    if (g_tick % 50 != 0) {
        return;
    }

    MID_OLED_Clear();

    /* Let app draw its content */
    g_apps[g_current].run();

    /* Overlay standard header on top */
    draw_header();

    MID_OLED_RefreshGram();
}

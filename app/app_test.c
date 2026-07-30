#include "app_test.h"
#include "bsp_servo.h"

/*
 * Button-triggered servo resolution / range test.
 *
 * Alternating on each key press:
 *   Odd press:  CCW positive, +1deg unit step (rack up a tiny bit)
 *   Even press: CW reverse,   large step back (rack down a lot)
 *
 * Position accumulates.  10ms TimerTick provides settling delay.
 *
 * Direction convention (verified with hardware):
 *   正转 = CCW = angle decreases = rack moves UP
 *   反转 = CW  = angle increases = rack moves DOWN
 */

typedef enum {
    SEQ_IDLE,
    SEQ_SETTLING,
} seq_state_t;

static seq_state_t s_state  = SEQ_IDLE;
static uint8_t     s_tick   = 0;
static int16_t     s_angle  = 135; /* tracked absolute position */
static bool        s_odd    = true; /* odd = tiny CCW, even = large CW */

#define UNIT_STEP      1    /*  1deg — smallest practical CCW step */
#define LARGE_CW_STEP  90   /* 90deg — large CW reversal */
#define SETTLE_TICKS   30   /* 300ms mechanical settling */

void APP_Test_Init(void)
{
    s_state = SEQ_IDLE;
    s_tick  = 0;
    s_angle = 135;
    s_odd   = true;
}

void APP_Test_TimerTick(void)
{
    if (s_state == SEQ_SETTLING) {
        s_tick++;
        if (s_tick >= SETTLE_TICKS) {
            s_state = SEQ_IDLE;
            s_tick  = 0;
        }
    }
}

void APP_Test_Run(void) {}

void APP_Test_Start(void)
{
    if (s_state != SEQ_IDLE) return;

    if (s_odd) {
        /* CCW positive: tiny unit step (rack up slightly) */
        s_angle -= UNIT_STEP;
        if (s_angle < 0) s_angle = 0;
    } else {
        /* CW reverse: large step (rack down a lot) */
        s_angle += LARGE_CW_STEP;
        if (s_angle > 270) s_angle = 270;
    }

    BSP_Servo_SetAngle((uint16_t)s_angle);
    s_odd    = !s_odd;
    s_state  = SEQ_SETTLING;
    s_tick   = 0;
}

void APP_Test_Stop(void) {}

bool APP_Test_IsRunning(void)
{
    return (s_state != SEQ_IDLE);
}

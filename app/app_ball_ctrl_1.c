#include "app_ball_ctrl_1.h"
#include "bsp_servo.h"
#include "mid_oled.h"

/*
 * Q3 open-loop bang-bang controller — timed sequence, no sensor.
 *
 * Strategy (large-angle → gravity dominates friction):
 *
 *   1. ACCEL:   CCW large angle  → ball rolls toward +5cm
 *   2. BRAKE1:  CW angle BEFORE ball reaches +5cm → decelerate
 *               so ball arrives at +5cm with v≈0
 *   3. COAST:   hold same CW angle as BRAKE1 → ball reverses,
 *               rolls back past 0 toward -5cm
 *   4. BRAKE2:  CCW angle BEFORE ball reaches -5cm → decelerate,
 *               ball stops at -5cm
 *   5. SETTLE:  hold near-center angle → stabilize ball at target
 *
 *   Total travel: O → +5cm → -5cm (≤5s, err ≤±1cm per competition)
 *
 * Direction (实测 — opposite of README, likely servo mounted reversed):
 *   CCW / 逆转 = angle < 135° = ball → +cm
 *   CW  / 正转 = angle > 135° = ball → -cm
 *
 *   offset sign in set_servo():
 *     +offset → CW  → ball → -cm (decelerate +cm motion)
 *     -offset → CCW → ball → +cm (accelerate +cm motion)
 *
 * Gear-rack kinematics (L = 244mm measured):
 *   Pipe tilt:  φ(deg) = -0.064 × Δθ_servo  (φ>0 = upward)
 *   Ball accel: a(m/s²) = -0.011 × Δθ_servo
 *   PWM:        PWM = 1500 + Δθ × 7.407
 *
 * All timing & angle values are tunable — tune with BLE !BALL log if
 * available, or by watching the ball and adjusting.
 */

/* ========== tunable parameters ========== */

#define SERVO_CENTER_DEG      135     /* horizontal (ball stable) */

/* --- Phase 1: accelerate right via CCW (ball → +cm) --- */
#define ACCEL_ANGLE           50      /* servo ° offset — set_servo(-ACCEL) = CCW */
#define ACCEL_TICKS           40      /* duration (×10ms = 400ms) */

/* --- Phase 2: brake BEFORE +5cm via CW (ball → -cm, opposing motion) --- */
#define BRAKE1_ANGLE          25      /* servo ° offset — set_servo(+BRAKE1) = CW */
#define BRAKE1_TICKS          25      /* duration (×10ms = 250ms) */

/* --- Phase 3: hold CW, ball rolls back past 0 toward -5cm (same angle as BRAKE1) --- */
#define COAST_TICKS           100     /* duration (×10ms = 1000ms), angle = BRAKE1_ANGLE */

/* --- Phase 4: brake BEFORE -5cm via CCW (ball → +cm, opposing motion) --- */
#define BRAKE2_ANGLE          40      /* servo ° offset — set_servo(-BRAKE2) = CCW */
#define BRAKE2_TICKS          40      /* duration (×10ms = 400ms) */

/* --- Phase 5: stabilize at target --- */
#define SETTLE_ANGLE          0       /* servo ° offset from center (0 = horizontal, tune if drifts) */
#define SETTLE_TICKS          50      /* hold before declaring done */

/* ========== internal types ========== */

typedef enum {
    STATE_IDLE,
    STATE_ACCEL,
    STATE_BRAKE1,
    STATE_COAST,
    STATE_BRAKE2,
    STATE_SETTLE,
    STATE_DONE,
} state_t;

/* ========== static variables ========== */

static state_t  s_state       = STATE_IDLE;
static uint16_t s_tick        = 0;    /* tick within current phase */
static uint16_t s_total_ticks = 0;    /* total elapsed since Start */
static uint16_t s_disp_tick   = 0;

/* ========== helpers ========== */

static void set_servo(int16_t offset_deg)
{
    int16_t angle = SERVO_CENTER_DEG + offset_deg;
    if (angle < 0)   angle = 0;
    if (angle > 270) angle = 270;
    BSP_Servo_SetAngle(angle);
}

static const char *state_name(state_t s)
{
    switch (s) {
    case STATE_IDLE:   return "IDLE";
    case STATE_ACCEL:  return "ACCEL";
    case STATE_BRAKE1: return "BRAKE1";
    case STATE_COAST:  return "COAST";
    case STATE_BRAKE2: return "BRAKE2";
    case STATE_SETTLE: return "SETTLE";
    case STATE_DONE:   return "DONE";
    }
    return "?";
}

static void fmt_time(char *buf, uint16_t ticks)
{
    uint16_t cs = ticks;         /* centiseconds */
    uint16_t sec = cs / 100;
    uint16_t frac = cs % 100;
    buf[0] = '0' + (sec / 10) % 10;
    buf[1] = '0' + sec % 10;
    buf[2] = '.';
    buf[3] = '0' + (frac / 10) % 10;
    buf[4] = '0' + frac % 10;
    buf[5] = ' ';
    buf[6] = 's';
    buf[7] = '\0';
}

/* ========== public API ========== */

void APP_BallCtrl1_Init(void)
{
    s_state       = STATE_IDLE;
    s_tick        = 0;
    s_total_ticks = 0;
    s_disp_tick   = 0;
    BSP_Servo_SetAngle(SERVO_CENTER_DEG);

    MID_OLED_Clear();
    MID_OLED_ShowString(16, 0, "Q3 BANG-BANG", 16);
    MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
    MID_OLED_ShowString(0, 38, "Key = Start", 12);
    MID_OLED_RefreshGram();
}

void APP_BallCtrl1_TimerTick(void)
{
    if (s_state == STATE_IDLE || s_state == STATE_DONE) {
        return;
    }

    s_total_ticks++;
    s_disp_tick++;
    s_tick++;

    /* State transitions are purely time-driven */
    switch (s_state) {

    case STATE_ACCEL:
        if (s_tick >= ACCEL_TICKS) {
            s_state = STATE_BRAKE1;
            s_tick  = 0;
            set_servo(BRAKE1_ANGLE);
        }
        break;

    case STATE_BRAKE1:
        if (s_tick >= BRAKE1_TICKS) {
            s_state = STATE_COAST;
            s_tick  = 0;
            set_servo(BRAKE1_ANGLE);  /* hold same angle as BRAKE1, ball reverses */
        }
        break;

    case STATE_COAST:
        if (s_tick >= COAST_TICKS) {
            s_state = STATE_BRAKE2;
            s_tick  = 0;
            set_servo(-BRAKE2_ANGLE);
        }
        break;

    case STATE_BRAKE2:
        if (s_tick >= BRAKE2_TICKS) {
            s_state = STATE_SETTLE;
            s_tick  = 0;
            set_servo(SETTLE_ANGLE);  /* stabilize at target, tune if ball drifts */
        }
        break;

    case STATE_SETTLE:
        if (s_tick >= SETTLE_TICKS) {
            s_state = STATE_DONE;
            s_tick  = 0;
        }
        break;

    default:
        break;
    }
}

void APP_BallCtrl1_Run(void)
{
    char buf[16];

    if (s_disp_tick % 10 != 0) return;  /* ~10 Hz */

    MID_OLED_Clear();

    switch (s_state) {

    case STATE_IDLE:
        MID_OLED_ShowString(16, 0, "Q3 BANG-BANG", 16);
        MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
        MID_OLED_ShowString(0, 38, "Key = Start", 12);
        break;

    case STATE_ACCEL:
        MID_OLED_ShowString(28, 0, "Q3 -> +5 cm", 16);
        MID_OLED_ShowString(0, 22, "Phase: ACCEL", 12);
        MID_OLED_ShowString(0, 34, "Servo: -", 12);
        MID_OLED_ShowNumber(54, 34, ACCEL_ANGLE, 3, 12);
        MID_OLED_ShowString(78, 34, "deg", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 48, buf, 12);
        break;

    case STATE_BRAKE1:
        MID_OLED_ShowString(28, 0, "Q3 -> +5 cm", 16);
        MID_OLED_ShowString(0, 22, "Phase: BRAKE1", 12);
        MID_OLED_ShowString(0, 34, "Servo: +", 12);
        MID_OLED_ShowNumber(54, 34, BRAKE1_ANGLE, 3, 12);
        MID_OLED_ShowString(78, 34, "deg", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 48, buf, 12);
        break;

    case STATE_COAST:
        MID_OLED_ShowString(28, 0, "Q3 <- BACK", 16);
        MID_OLED_ShowString(0, 22, "Phase: COAST", 12);
        MID_OLED_ShowString(0, 34, "Servo: +", 12);
        MID_OLED_ShowNumber(54, 34, BRAKE1_ANGLE, 3, 12);
        MID_OLED_ShowString(78, 34, "deg", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 48, buf, 12);
        break;

    case STATE_BRAKE2:
        MID_OLED_ShowString(28, 0, "Q3 -> -5 cm", 16);
        MID_OLED_ShowString(0, 22, "Phase: BRAKE2", 12);
        MID_OLED_ShowString(0, 34, "Servo: -", 12);
        MID_OLED_ShowNumber(54, 34, BRAKE2_ANGLE, 3, 12);
        MID_OLED_ShowString(78, 34, "deg", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 48, buf, 12);
        break;

    case STATE_SETTLE:
        MID_OLED_ShowString(22, 0, "Q3 SETTLE", 16);
        MID_OLED_ShowString(0, 34, "Servo:", 12);
        MID_OLED_ShowNumber(48, 34, SETTLE_ANGLE, 3, 12);
        MID_OLED_ShowString(72, 34, "deg", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 48, buf, 12);
        break;

    case STATE_DONE:
        MID_OLED_ShowString(28, 0, "Q3  DONE!", 16);
        MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 38, "Time:", 12);
        MID_OLED_ShowString(48, 38, buf, 12);
        break;
    }

    MID_OLED_RefreshGram();
}

void APP_BallCtrl1_Start(void)
{
    if (s_state != STATE_IDLE && s_state != STATE_DONE) return;

    s_state       = STATE_ACCEL;
    s_tick        = 0;
    s_total_ticks = 0;
    s_disp_tick   = 0;
    set_servo(-ACCEL_ANGLE);
}

void APP_BallCtrl1_Stop(void)
{
    s_state = STATE_IDLE;
    BSP_Servo_SetAngle(SERVO_CENTER_DEG);
}

bool APP_BallCtrl1_IsRunning(void)
{
    return (s_state != STATE_IDLE && s_state != STATE_DONE);
}

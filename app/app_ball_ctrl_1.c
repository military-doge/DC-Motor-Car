#include "app_ball_ctrl_1.h"
#include "bsp_servo.h"
#include "bsp_delay.h"
#include "mid_k230.h"
#include "mid_oled.h"
#include <stdlib.h>
#include <math.h>

/*
 * Q3 pure visual closed-loop PID controller.
 *
 * Sequence: IDLE → TO_P5 (O→+5cm) → TO_M5 (+5cm→-5cm) → DONE
 * Each phase uses full PID authority — no open-loop base angle.
 * State transitions driven by position arrival, not time.
 *
 * Control law (per README authoritative formula):
 *   error = target - pos         (pos from MID_K230_GetPosition(), mm)
 *   correction = KP * error      (KP < 0, empirically verified direction)
 *   servo_offset += correction   (incremental accumulation = integral action)
 *
 * Direction (实测, 权威方向表):
 *   CCW / 逆转 = angle < 135° = ball → +cm
 *   CW  / 正转 = angle > 135° = ball → -cm
 *   +offset → CW  → ball → -cm
 *   -offset → CCW → ball → +cm
 *
 * Gear-rack kinematics (L = 244mm measured):
 *   Pipe tilt:  φ(deg) = +0.064 × Δθ_servo  (φ>0 = 上倾)
 *   Ball accel: a(m/s²) = +0.011 × Δθ_servo  (+a = 球→-cm)
 *   PWM:        PWM = 1500 + Δθ × 7.407
 */

/* ========== tunable parameters ========== */

#define SERVO_CENTER_DEG      135     /* horizontal */

/* --- Vision PID gains (实测验证方向, KP/KI/KD < 0) --- */
#define VISION_KP              -2.885f /* P gain deg/mm */
#define VISION_KI              -0.096f /* I gain deg/mm/s */
#define VISION_KD              -2.290f /* D gain deg per mm/s (微分先行) */
#define VISION_I_MAX            43.0f  /* integral clamp ±deg */
#define VISION_I_ERR_THR        20.0f  /* integral separation: only integrate when |error| < this */
#define VISION_DATA_TIMEOUT_MS 200     /* K230 data age threshold ms */

/* --- Dither (high-frequency micro-vibration to break static friction) --- */
#define DITHER_AMP_DEG          0.0f   /* dither amplitude deg (disabled) */
#define DITHER_FREQ_HZ          12.0f  /* dither frequency Hz */

/* --- Stuck detection (ball not moving despite servo action) --- */
#define STUCK_VEL_THR_MM_S      5.0f   /* velocity below this = stuck (mm/s) */
#define STUCK_POS_THR_MM        20.0f  /* position far from target (mm) */
#define STUCK_BOOST_DEG         7.68f  /* extra deg when stuck detected */

/* --- PID output clamp (±deg, symmetric) --- */
#define PID_CLAMP_DEG           29.3f  /* full PID authority for movement */

/* --- Fine-tuning softener: scale down P+I near target, D always full --- */
#define FINE_ERR_THR_MM          20.0f  /* error below this → begin scaling P+I */
#define FINE_SCALE_MIN           0.35f  /* minimum scale factor at error=0 */

/* --- Phase targets and arrival detection --- */
#define TARGET_P5_MM            50     /* +5cm */
#define TARGET_M5_MM           -50     /* -5cm */
#define ARRIVE_THR_MM           10     /* ±10mm = arrived per competition ±1cm */
#define ARRIVE_COUNT            10     /* consecutive frames to confirm arrival */
#define HOLD_TICKS              50     /* hold at -5cm for 0.5s before DONE */
#define HOLD_VEL_THR_MM_S        3.0f   /* ball must be slower than this to count as stationary */
#define HOLD_DEADBAND_MM         6.0f   /* |error| < this → freeze servo, no more adjustment */
#define TOTAL_TIMEOUT_TICKS     800    /* 8s total timeout (was 5s) */
#define DONE_HOLD_TICKS          500    /* 5s post-DONE servo hold before centering */

/* ========== internal types ========== */

typedef enum {
    STATE_IDLE,
    STATE_TO_P5,     /* moving ball from O → +5cm */
    STATE_TO_M5,     /* moving ball from +5cm → -5cm */
    STATE_HOLD,      /* holding at -5cm for 1s before DONE */
    STATE_DONE,
} state_t;

/* ========== static variables ========== */

static state_t  s_state       = STATE_IDLE;
static uint16_t s_tick        = 0;    /* tick within current phase */
static uint16_t s_total_ticks = 0;    /* total elapsed since Start */
static uint16_t s_disp_tick   = 0;

/* ---- Vision PID state ---- */
static float    s_prev_k230_pos;        /* previous K230 position mm */
static uint32_t s_prev_k230_ts;         /* previous K230 timestamp ms */
static float    s_i_accum       = 0.0f; /* integral accumulator deg */
static float    s_dither_phase  = 0.0f; /* dither phase rad */
static float    s_ball_velocity = 0.0f; /* ball velocity mm/s */
static float    s_vision_trim   = 0.0f; /* latest vision trim deg */
static float    s_k230_pos      = 0.0f; /* latest K230 position mm */
static float    s_k230_error    = 0.0f; /* latest error mm */
static bool     s_k230_has_prev = false;/* whether previous frame exists for dt calc */
static uint8_t  s_arrive_cnt    = 0;    /* consecutive within-threshold frames */
static uint16_t s_hold_ticks    = 0;    /* ticks held at -5cm within threshold */
static float    s_target_mm     = 0.0f; /* current phase target mm */
static int16_t  s_servo_angle   = 135;   /* current servo angle deg (for display) */

/* ========== helpers ========== */

static void set_servo(int16_t offset_deg)
{
    int16_t angle = SERVO_CENTER_DEG + offset_deg;
    if (angle < 0)   angle = 0;
    if (angle > 270) angle = 270;
    s_servo_angle = angle;
    BSP_Servo_SetAngle(angle);
}

/*
 * Unified vision PID trim computation.
 * Returns trim clipped to ±clamp_deg, or 0.0f if K230 data unavailable.
 */
static float compute_vision_trim(float target_mm, float clamp_deg)
{
    float    pos, error_mm, dt, vel, abs_err;
    float    p_term, d_term, dither, stuck_boost, abs_vel, trim;
    uint32_t age, k230_ts;
    int32_t  diff;

    /* ---- check K230 data freshness ---- */
    age = BSP_Delay_GetTick() - MID_K230_GetLastUpdate();
    if (!(MID_K230_IsDetected() && (MID_K230_GetLastUpdate() > 0)
            && (age < VISION_DATA_TIMEOUT_MS))) {
        return 0.0f;
    }

    pos      = MID_K230_GetPosition();
    k230_ts  = MID_K230_GetTimestamp();
    error_mm = target_mm - pos;

    /* ---- dt from K230 timestamps (ms → s), velocity (mm/s) ---- */
    diff = (int32_t)(k230_ts - s_prev_k230_ts);
    if (s_k230_has_prev && diff > 0 && diff < 500) {
        /* New K230 frame: compute velocity from actual dt */
        dt = (float)diff / 1000.0f;
        vel = (pos - s_prev_k230_pos) / dt;
        s_ball_velocity = vel;
        s_prev_k230_pos = pos;
        s_prev_k230_ts  = k230_ts;
    } else if (s_k230_has_prev) {
        /* Same frame: keep previous velocity, don't reset to 0 */
        vel = s_ball_velocity;
    } else {
        /* First frame: no velocity yet */
        vel = 0.0f;
        s_ball_velocity = 0.0f;
        s_prev_k230_pos = pos;
        s_prev_k230_ts  = k230_ts;
        s_k230_has_prev = true;
    }

    s_k230_pos      = pos;
    s_k230_error    = error_mm;

    /* ---- P term ---- */
    p_term = VISION_KP * error_mm;

    /* ---- I term (conditional accumulation + anti-windup) ---- */
    abs_err = error_mm < 0 ? -error_mm : error_mm;
    if (abs_err < VISION_I_ERR_THR) {
        s_i_accum += VISION_KI * error_mm * dt;
        if (s_i_accum > +VISION_I_MAX) s_i_accum = +VISION_I_MAX;
        if (s_i_accum < -VISION_I_MAX) s_i_accum = -VISION_I_MAX;
    }

    /* ---- D term (derivative-on-measurement) ---- */
    d_term = VISION_KD * (-vel);

    /* ---- Dither ---- */
    dither = DITHER_AMP_DEG * (float)sin((double)s_dither_phase);

    /* ---- Stuck detection ---- */
    stuck_boost = 0.0f;
    abs_vel = vel < 0 ? -vel : vel;
    if (abs_vel < STUCK_VEL_THR_MM_S && abs_err > STUCK_POS_THR_MM && s_k230_has_prev) {
        stuck_boost = (error_mm > 0) ? -STUCK_BOOST_DEG : +STUCK_BOOST_DEG;
    }

    /* ---- Sum and clip ---- */
    trim = p_term + s_i_accum + d_term + dither + stuck_boost;

    if (trim > +clamp_deg) trim = +clamp_deg;
    if (trim < -clamp_deg) trim = -clamp_deg;

    /* ---- Fine-tuning softener: scale down near target ---- */
    {
        float abs_err_fine = error_mm < 0 ? -error_mm : error_mm;
        float scale;
        if (abs_err_fine >= FINE_ERR_THR_MM) {
            scale = 1.0f;
        } else {
            scale = FINE_SCALE_MIN
                  + (1.0f - FINE_SCALE_MIN) * (abs_err_fine / FINE_ERR_THR_MM);
        }
        trim *= scale;
    }

    return trim;
}

/*
 * Reset vision PID state for phase transition.
 */
static void vision_reset(void)
{
    s_i_accum        = 0.0f;
    s_dither_phase   = 0.0f;
    s_ball_velocity  = 0.0f;
    s_vision_trim    = 0.0f;
    s_k230_pos       = 0.0f;
    s_k230_error     = 0.0f;
    s_k230_has_prev  = false;
    s_prev_k230_pos  = 0.0f;
    s_prev_k230_ts   = 0;
    s_arrive_cnt     = 0;
    s_hold_ticks     = 0;
}

static const char *state_name(state_t s)
{
    switch (s) {
    case STATE_IDLE:  return "IDLE";
    case STATE_TO_P5: return "TO_P5";
    case STATE_TO_M5: return "TO_M5";
    case STATE_HOLD: return "HOLD";
    case STATE_DONE:  return "DONE";
    }
    return "?";
}

static void fmt_time(char *buf, uint16_t ticks)
{
    uint16_t cs = ticks;
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

static void show_k230_pos(uint8_t x, uint8_t y)
{
    char buf[10];
    if (MID_K230_IsDetected()) {
        float    pos = MID_K230_GetPosition();
        int32_t  v   = (int32_t)(pos >= 0 ? pos + 0.5f : pos - 0.5f);
        uint8_t  p   = 0;
        buf[p++] = (v < 0) ? '-' : '+';
        if (v < 0) v = -v;
        buf[p++] = '0' + (v / 100) % 10;
        buf[p++] = '0' + (v / 10) % 10;
        buf[p++] = '0' + (v % 10);
        buf[p++] = 'm'; buf[p++] = 'm'; buf[p] = '\0';
        MID_OLED_ShowString(x, y, buf, 12);
    } else {
        MID_OLED_ShowString(x, y, "  -- mm", 12);
    }
}

static void show_vision_trim(uint8_t x, uint8_t y)
{
    char buf[7];
    int16_t v = (int16_t)(s_vision_trim >= 0 ? s_vision_trim + 0.5f : s_vision_trim - 0.5f);
    uint8_t p = 0;
    if (v < 0) { buf[p++] = '-'; v = (int16_t)(-v); }
    else       { buf[p++] = '+'; }
    if (v >= 100)      { buf[p++] = '0' + (v / 100) % 10; }
    if (v >= 10)       { buf[p++] = '0' + (v / 10) % 10; }
    buf[p++] = '0' + (v % 10);
    buf[p++] = 'd';
    buf[p]   = '\0';
    MID_OLED_ShowString(x, y, buf, 12);
}

/* Display target mm (e.g. "+050" or "-050") */
static void show_target(uint8_t x, uint8_t y, int16_t target_mm)
{
    char buf[5];
    uint8_t p = 0;
    int16_t v = target_mm;
    buf[p++] = (v < 0) ? '-' : '+';
    if (v < 0) v = (int16_t)(-v);
    buf[p++] = '0' + (v / 100) % 10;
    buf[p++] = '0' + (v / 10) % 10;
    buf[p++] = '0' + (v % 10);
    buf[p]   = '\0';
    MID_OLED_ShowString(x, y, buf, 12);
}

/* ========== public API ========== */

void APP_BallCtrl1_Init(void)
{
    s_state       = STATE_IDLE;
    s_tick        = 0;
    s_total_ticks = 0;
    s_disp_tick   = 0;
    s_target_mm   = 0.0f;
    vision_reset();
    BSP_Servo_SetAngle(SERVO_CENTER_DEG);

    MID_OLED_Clear();
    MID_OLED_ShowString(16, 0, "Q3 CLOSED-LOOP", 16);
    MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
    MID_OLED_ShowString(0, 38, "Key = Start", 12);
    MID_OLED_RefreshGram();
}

void APP_BallCtrl1_TimerTick(void)
{
    if (s_state == STATE_IDLE) {
        return;
    }

    /* STATE_DONE: keep servo active for 5s post-completion */
    if (s_state == STATE_DONE) {
        s_tick++;
        if (s_tick >= DONE_HOLD_TICKS) {
            set_servo(0);
            s_state = STATE_IDLE;  /* truly done */
        } else {
            /* Continue PID for 5s to hold ball at final position */
            float trim = compute_vision_trim(s_target_mm, PID_CLAMP_DEG);
            s_vision_trim = trim;
            set_servo((int16_t)trim);
        }
        return;
    }

    s_total_ticks++;
    s_disp_tick++;
    s_tick++;

    /* Advance dither phase each tick (100Hz) */
    s_dither_phase += 2.0f * 3.14159f * DITHER_FREQ_HZ / 100.0f;
    if (s_dither_phase > 6.28318f) s_dither_phase -= 6.28318f;

    /* Global timeout */
    if (s_total_ticks >= TOTAL_TIMEOUT_TICKS) {
        s_state = STATE_DONE;
        s_tick  = 0;
        /* 5s hold then center (handled in STATE_DONE branch above) */
        return;
    }

    float   trim;
    int16_t target;

    switch (s_state) {

    case STATE_TO_P5:
        /*
         * Pure PID: drive ball from O to +5cm.
         * No open-loop base. PID + dither + stuck detection.
         */
        target = TARGET_P5_MM;
        s_target_mm = (float)target;
        trim   = compute_vision_trim((float)target, PID_CLAMP_DEG);
        s_vision_trim = trim;
        set_servo((int16_t)trim);

        /* Arrival detection */
        {
            float abs_err = s_k230_error < 0 ? -s_k230_error : s_k230_error;
            if (abs_err < (float)ARRIVE_THR_MM) {
                s_arrive_cnt++;
                if (s_arrive_cnt >= ARRIVE_COUNT) {
                    /* Arrived at +5cm → switch to TO_M5 */
                    s_state = STATE_TO_M5;
                    s_tick  = 0;
                    vision_reset();  /* fresh PID for reverse direction */
                }
            } else {
                s_arrive_cnt = 0;
            }
        }
        break;

    case STATE_TO_M5:
        /*
         * Pure PID: drive ball from +5cm to -5cm.
         */
        target = TARGET_M5_MM;
        s_target_mm = (float)target;
        trim   = compute_vision_trim((float)target, PID_CLAMP_DEG);
        s_vision_trim = trim;
        set_servo((int16_t)trim);

        /* Arrival detection */
        {
            float abs_err = s_k230_error < 0 ? -s_k230_error : s_k230_error;
            if (abs_err < (float)ARRIVE_THR_MM) {
                s_arrive_cnt++;
                if (s_arrive_cnt >= ARRIVE_COUNT) {
                    s_state = STATE_HOLD;
                    s_tick  = 0;
                    s_hold_ticks = 0;
                }
            } else {
                s_arrive_cnt = 0;
            }
        }
        break;

    case STATE_HOLD:
        /*
         * Hold at -5cm: PID live, but freeze servo if ball is
         * within deadband + stationary for HOLD_TICKS to complete.
         */
        target = TARGET_M5_MM;
        s_target_mm = (float)target;
        trim   = compute_vision_trim((float)target, PID_CLAMP_DEG);
        s_vision_trim = trim;

        {
            float abs_err = s_k230_error < 0 ? -s_k230_error : s_k230_error;
            float abs_vel = s_ball_velocity < 0 ? -s_ball_velocity : s_ball_velocity;

            if (abs_err < HOLD_DEADBAND_MM && abs_vel < HOLD_VEL_THR_MM_S) {
                /* Ball stable in deadband: freeze servo, count hold */
                s_hold_ticks++;
                if (s_hold_ticks >= HOLD_TICKS) {
                    s_state = STATE_DONE;
                    s_tick  = 0;
                    /* Don't center yet — 5s hold then center (handled in TimerTick) */
                }
            } else if (abs_err < (float)ARRIVE_THR_MM && abs_vel < HOLD_VEL_THR_MM_S) {
                /* Within arrival zone, not deadband: keep adjusting */
                s_hold_ticks = 0;
                set_servo((int16_t)trim);
            } else {
                /* Ball moved: reset, keep adjusting */
                s_hold_ticks = 0;
                set_servo((int16_t)trim);
            }
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
        MID_OLED_ShowString(16, 0, "Q3 CLOSED-LOOP", 16);
        MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
        MID_OLED_ShowString(0, 38, "SA:", 12);
        MID_OLED_ShowNumber(18, 38, (uint32_t)s_servo_angle, 3, 12);
        MID_OLED_ShowString(0, 52, "Key = Start", 12);
        break;

    case STATE_TO_P5:
        MID_OLED_ShowString(28, 0, "Q3 -> +5 cm", 16);
        /* Row 1: position | target */
        MID_OLED_ShowString(0, 16, "P:", 12);
        show_k230_pos(18, 16);
        MID_OLED_ShowString(84, 16, "T:", 12);
        show_target(96, 16, TARGET_P5_MM);
        /* Row 2: error | trim */
        MID_OLED_ShowString(0, 28, "E:", 12);
        {
            int32_t v = (int32_t)(s_k230_error >= 0 ? s_k230_error + 0.5f : s_k230_error - 0.5f);
            char tbuf[7];
            uint8_t p = 0;
            if (v < 0) { tbuf[p++] = '-'; v = -v; }
            else       { tbuf[p++] = '+'; }
            tbuf[p++] = '0' + (v / 10) % 10;
            tbuf[p++] = '0' + (v % 10);
            tbuf[p++] = 'm'; tbuf[p++] = 'm'; tbuf[p] = '\0';
            MID_OLED_ShowString(18, 28, tbuf, 12);
        }
        MID_OLED_ShowString(54, 28, "V:", 12);
        show_vision_trim(72, 28);
        /* Row 3: servo angle | I accum */
        MID_OLED_ShowString(0, 40, "SA:", 12);
        MID_OLED_ShowNumber(18, 40, (uint32_t)s_servo_angle, 3, 12);
        {
            int32_t v = (int32_t)(s_i_accum >= 0 ? s_i_accum + 0.5f : s_i_accum - 0.5f);
            char tbuf[7];
            uint8_t p = 0;
            MID_OLED_ShowString(54, 40, "I:", 12);
            if (v < 0) { tbuf[p++] = '-'; v = -v; }
            else       { tbuf[p++] = '+'; }
            tbuf[p++] = '0' + (v / 10) % 10;
            tbuf[p++] = '0' + (v % 10);
            tbuf[p++] = 'd'; tbuf[p] = '\0';
            MID_OLED_ShowString(72, 40, tbuf, 12);
        }
        /* Row 4: arrive count | time */
        MID_OLED_ShowString(0, 52, "C:", 12);
        MID_OLED_ShowNumber(12, 52, s_arrive_cnt, 2, 12);
        MID_OLED_ShowString(24, 52, "/", 12);
        MID_OLED_ShowNumber(30, 52, ARRIVE_COUNT, 2, 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(60, 52, buf, 12);
        break;

    case STATE_TO_M5:
        MID_OLED_ShowString(28, 0, "Q3 -> -5 cm", 16);
        /* Row 1: position | target */
        MID_OLED_ShowString(0, 16, "P:", 12);
        show_k230_pos(18, 16);
        MID_OLED_ShowString(84, 16, "T:", 12);
        show_target(96, 16, TARGET_M5_MM);
        /* Row 2: error | trim */
        MID_OLED_ShowString(0, 28, "E:", 12);
        {
            int32_t v = (int32_t)(s_k230_error >= 0 ? s_k230_error + 0.5f : s_k230_error - 0.5f);
            char tbuf[7];
            uint8_t p = 0;
            if (v < 0) { tbuf[p++] = '-'; v = -v; }
            else       { tbuf[p++] = '+'; }
            tbuf[p++] = '0' + (v / 10) % 10;
            tbuf[p++] = '0' + (v % 10);
            tbuf[p++] = 'm'; tbuf[p++] = 'm'; tbuf[p] = '\0';
            MID_OLED_ShowString(18, 28, tbuf, 12);
        }
        MID_OLED_ShowString(54, 28, "V:", 12);
        show_vision_trim(72, 28);
        /* Row 3: servo angle | I accum */
        MID_OLED_ShowString(0, 40, "SA:", 12);
        MID_OLED_ShowNumber(18, 40, (uint32_t)s_servo_angle, 3, 12);
        {
            int32_t v = (int32_t)(s_i_accum >= 0 ? s_i_accum + 0.5f : s_i_accum - 0.5f);
            char tbuf[7];
            uint8_t p = 0;
            MID_OLED_ShowString(54, 40, "I:", 12);
            if (v < 0) { tbuf[p++] = '-'; v = -v; }
            else       { tbuf[p++] = '+'; }
            tbuf[p++] = '0' + (v / 10) % 10;
            tbuf[p++] = '0' + (v % 10);
            tbuf[p++] = 'd'; tbuf[p] = '\0';
            MID_OLED_ShowString(72, 40, tbuf, 12);
        }
        /* Row 4: arrive count | time */
        MID_OLED_ShowString(0, 52, "C:", 12);
        MID_OLED_ShowNumber(12, 52, s_arrive_cnt, 2, 12);
        MID_OLED_ShowString(24, 52, "/", 12);
        MID_OLED_ShowNumber(30, 52, ARRIVE_COUNT, 2, 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(60, 52, buf, 12);
        break;

    case STATE_HOLD:
        MID_OLED_ShowString(22, 0, "Q3 HOLD -5 cm", 16);
        /* Row 1: position | target */
        MID_OLED_ShowString(0, 16, "P:", 12);
        show_k230_pos(18, 16);
        MID_OLED_ShowString(84, 16, "T:", 12);
        show_target(96, 16, TARGET_M5_MM);
        /* Row 2: error | trim */
        MID_OLED_ShowString(0, 28, "E:", 12);
        {
            int32_t v = (int32_t)(s_k230_error >= 0 ? s_k230_error + 0.5f : s_k230_error - 0.5f);
            char tbuf[7];
            uint8_t p = 0;
            if (v < 0) { tbuf[p++] = '-'; v = -v; }
            else       { tbuf[p++] = '+'; }
            tbuf[p++] = '0' + (v / 10) % 10;
            tbuf[p++] = '0' + (v % 10);
            tbuf[p++] = 'm'; tbuf[p++] = 'm'; tbuf[p] = '\0';
            MID_OLED_ShowString(18, 28, tbuf, 12);
        }
        MID_OLED_ShowString(54, 28, "V:", 12);
        show_vision_trim(72, 28);
        /* Row 3: servo angle | hold count */
        MID_OLED_ShowString(0, 40, "SA:", 12);
        MID_OLED_ShowNumber(18, 40, (uint32_t)s_servo_angle, 3, 12);
        MID_OLED_ShowString(54, 40, "H:", 12);
        MID_OLED_ShowNumber(72, 40, s_hold_ticks, 3, 12);
        /* Row 4: hold / required | time */
        MID_OLED_ShowString(0, 52, "Hold:", 12);
        MID_OLED_ShowNumber(36, 52, s_hold_ticks, 3, 12);
        MID_OLED_ShowString(54, 52, "/", 12);
        MID_OLED_ShowNumber(60, 52, HOLD_TICKS, 3, 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(84, 52, buf, 12);
        break;

    case STATE_DONE: {
        MID_OLED_ShowString(28, 0, "Q3  DONE!", 16);
        if (MID_K230_IsDetected()) {
            char dbuf[8];
            int32_t ferr = (int32_t)(s_k230_error >= 0 ? s_k230_error + 0.5f : s_k230_error - 0.5f);
            uint8_t p = 0;
            MID_OLED_ShowString(0, 22, "Final err:", 12);
            if (ferr < 0) { dbuf[p++] = '-'; ferr = -ferr; }
            else          { dbuf[p++] = '+'; }
            dbuf[p++] = '0' + (ferr / 10) % 10;
            dbuf[p++] = '0' + (ferr % 10);
            dbuf[p++] = 'm'; dbuf[p++] = 'm'; dbuf[p] = '\0';
            MID_OLED_ShowString(66, 22, dbuf, 12);
        } else {
            MID_OLED_ShowString(0, 22, "O -> +5 -> -5 cm", 12);
        }
        MID_OLED_ShowString(0, 38, "SA:", 12);
        MID_OLED_ShowNumber(18, 38, (uint32_t)s_servo_angle, 3, 12);
        fmt_time(buf, s_total_ticks);
        MID_OLED_ShowString(0, 52, "Time:", 12);
        MID_OLED_ShowString(48, 52, buf, 12);
        break;
    }
    }

    MID_OLED_RefreshGram();
}

void APP_BallCtrl1_Start(void)
{
    if (s_state != STATE_IDLE && s_state != STATE_DONE) return;

    s_state       = STATE_TO_P5;
    s_tick        = 0;
    s_total_ticks = 0;
    s_disp_tick   = 0;
    s_target_mm   = (float)TARGET_P5_MM;
    vision_reset();  /* fresh PID + frame history */
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

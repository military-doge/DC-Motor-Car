#include "bsp_servo.h"
#include "ti_msp_dl_config.h"

/*
 * S20F 270° digital servo PWM control (TIMA0, PA0).
 *
 * SysConfig baseline (DC-Motor-Car.syscfg → ti_msp_dl_config.c):
 *   TIMA0, BUSCLK=80MHz, prescale=78 → clk ≈ 1.01266 MHz
 *   period = 19999 → ～50.6 Hz (≈20 ms), edge-align-up
 *   CCP0 initHigh, startTimer = DL_TIMER_START
 *
 * The SysConfig init sets CC=0 (output held LOW).  This module:
 *   1. Stops and restarts the counter to force a clean edge,
 *   2. Writes the desired pulse width to CCP0.
 */

bool BSP_Servo_Init(void)
{
    /*
     * SysConfig auto-starts TIMA0, but the counter may be mid-cycle
     * and/or the CC shadow register may be stale.  Stop -> set -> start
     * guarantees a predictable first edge.
     */
    DL_TimerA_stopCounter(PWM_SERVO_INST);

    /* Re-confirm CCP0 is output */
    DL_TimerA_setCCPDirection(PWM_SERVO_INST, DL_TIMER_CC0_OUTPUT);

    /*
     * PA0 is open-drain on this board — external 4.7kΩ pull-up to 3.3V
     * already soldered. No internal pull-up needed.
     */
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_SERVO_C0_IOMUX, GPIO_PWM_SERVO_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_SERVO_C0_PORT, GPIO_PWM_SERVO_C0_PIN);

    BSP_Servo_SetAngle(135);

    DL_TimerA_startCounter(PWM_SERVO_INST);

    return true;
}

void BSP_Servo_SetAngle(uint16_t angle_deg)
{
    uint32_t pulse_us;

    if (angle_deg > 270) {
        angle_deg = 270;
    }

    /* PWM_us = 500 + angle * 2000 / 270  (0° → 500 us, 135° → 1500 us, 270° → 2500 us) */
    pulse_us = 500 + (uint32_t)angle_deg * 2000 / 270;

    /* Use TimerA-specific API — same pattern as WHEELTEC motor.c Set_PWM() */
    DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST,
        pulse_us, GPIO_PWM_SERVO_C0_IDX);
}

void BSP_Servo_SetPWM(uint16_t pulse_us)
{
    if (pulse_us < 500) {
        pulse_us = 500;
    }
    if (pulse_us > 2500) {
        pulse_us = 2500;
    }

    DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST,
        pulse_us, GPIO_PWM_SERVO_C0_IDX);
}

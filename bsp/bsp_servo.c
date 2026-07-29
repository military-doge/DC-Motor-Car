#include "bsp_servo.h"
#include "ti_msp_dl_config.h"

bool BSP_Servo_Init(void)
{
    /*
     * Re-confirm PA0 IOMUX: set as TIMA0 CCP0 output.
     * PA1 is released to digital input (MPU6050 SCL conflict on S28A).
     */
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_SERVO_C0_IOMUX, GPIO_PWM_SERVO_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_SERVO_C0_PORT, GPIO_PWM_SERVO_C0_PIN);
    DL_GPIO_initDigitalInput(GPIO_PWM_SERVO_C1_IOMUX);

    /*
     * SysConfig sets prescale=0 (80MHz tick → 4kHz PWM).
     * Override to prescale=79: 80MHz/(79+1)=1MHz → 50Hz, 1μs resolution.
     * Must stop timer before clock change (TIMA shadow register limitation).
     */
    DL_TimerA_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 79U
    };

    DL_Timer_stopCounter(PWM_SERVO_INST);
    DL_TimerA_setClockConfig(PWM_SERVO_INST, &clockCfg);

    /* Full PWM re-init after clock change */
    DL_TimerA_PWMConfig pwmCfg = {
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .period            = 20000,
        .isTimerWithFourCC = true,
        .startTimer        = DL_TIMER_START
    };
    DL_TimerA_initPWMMode(PWM_SERVO_INST, &pwmCfg);

    DL_TimerA_setCaptCompUpdateMethod(PWM_SERVO_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    /* Only CCP0 output; CCP1 stays internal */
    DL_TimerA_setCCPDirection(PWM_SERVO_INST, DL_TIMER_CC0_OUTPUT);

    BSP_Servo_SetAngle(135);

    return true;
}

void BSP_Servo_SetAngle(uint16_t angle_deg)
{
    uint32_t pulse_us;

    if (angle_deg > 270) {
        angle_deg = 270;
    }

    /* PWM_us = 500 + angle * 2000/270  (0°→500μs, 270°→2500μs) */
    pulse_us = 500 + (uint32_t)angle_deg * 2000 / 270;

    DL_Timer_setCaptureCompareValue(PWM_SERVO_INST,
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

    DL_Timer_setCaptureCompareValue(PWM_SERVO_INST,
        pulse_us, GPIO_PWM_SERVO_C0_IDX);
}

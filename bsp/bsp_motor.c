#include "bsp_motor.h"
#include "ti_msp_dl_config.h"
#include <stdlib.h>

bool BSP_Motor_Init(void)
{
    /* Start PWM counter (timer configured by SysConfig, auto-started) */
    DL_Timer_startCounter(PWM_0_INST);

    /* Initialize both motors stopped: direction pins low, PWM duty = 0 */
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);

    return true;
}

static int16_t motor_limit_pwm(int16_t value, int16_t low, int16_t high)
{
    if (value > high) return high;
    if (value < low)  return low;
    return value;
}

void BSP_Motor_SetPWM(int16_t pwm_a, int16_t pwm_b)
{
    int16_t abs_a = abs(pwm_a);
    int16_t abs_b = abs(pwm_b);

    abs_a = motor_limit_pwm(abs_a, 0, 8000);
    abs_b = motor_limit_pwm(abs_b, 0, 8000);

    /* Motor A direction and duty */
    if (pwm_a > 0) {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, abs_a, GPIO_PWM_0_C0_IDX);
    } else if (pwm_a < 0) {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, abs_a, GPIO_PWM_0_C0_IDX);
    } else {
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    }

    /* Motor B direction and duty */
    if (pwm_b > 0) {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, abs_b, GPIO_PWM_0_C1_IDX);
    } else if (pwm_b < 0) {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, abs_b, GPIO_PWM_0_C1_IDX);
    } else {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
}

void BSP_Motor_Stop(void)
{
    BSP_Motor_SetPWM(0, 0);
}

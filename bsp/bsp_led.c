#include "bsp_led.h"

bool BSP_LED_Init(void)
{
    /* GPIO already configured by SYSCFG_DL_GPIO_init(), set initial state OFF */
    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    return true;
}

void BSP_LED_On(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
}

void BSP_LED_Off(void)
{
    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
}

void BSP_LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
}

void BSP_LED_Flash(uint16_t time)
{
    static uint16_t s_tick;
    if (time == 0) {
        s_tick = 0;
        BSP_LED_On();
    } else if (++s_tick >= time) {
        BSP_LED_Toggle();
        s_tick = 0;
    }
}

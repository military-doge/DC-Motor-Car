#include "bsp_delay.h"

static volatile uint32_t s_delay_millis;

/* SysTick ISR: 1ms tick (period = 80000 @ 80MHz) */
void SysTick_Handler(void)
{
    s_delay_millis++;
}

bool BSP_Delay_Init(void)
{
    s_delay_millis = 0;
    DL_SYSTICK_enableInterrupt();
    return true;
}

static uint32_t delay_millis(void)
{
    return s_delay_millis;
}

void BSP_Delay_ms(uint32_t ms)
{
    if (ms == 0) return;
    uint32_t start = delay_millis();
    while ((delay_millis() - start) < ms) { BSP_DELAY_WDT_FEED(); }
}

void BSP_Delay_us(uint32_t us)
{
    if (us == 0) return;

    /* Whole-millisecond portion: use the millis counter (no wraparound limit) */
    uint32_t ms = us / 1000;
    if (ms > 0) {
        uint32_t start = delay_millis();
        while ((delay_millis() - start) < ms) { BSP_DELAY_WDT_FEED(); }
    }

    /*
     * Sub-millisecond remainder: use SysTick->VAL directly.
     * Remainder is always < 1000 us and SysTick period is 1 ms (80000 ticks),
     * so at most one wraparound can occur.
     */
    uint32_t rem = us % 1000;
    if (rem > 0) {
        uint32_t target = rem * (CPUCLK_FREQ / 1000000UL);
        uint32_t start  = DL_SYSTICK_getValue();
        while (1) {
            uint32_t now     = DL_SYSTICK_getValue();
            uint32_t elapsed = (start >= now)
                ? (start - now)
                : (start + DL_SYSTICK_getPeriod() + 1U - now);
            if (elapsed >= target) break;
            BSP_DELAY_WDT_FEED();
        }
    }
}

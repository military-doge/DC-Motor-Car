#include "bsp_timer.h"
#include "ti_msp_dl_config.h"

static bsp_timer_callback_t s_callback = NULL;

bool BSP_Timer_Init(void)
{
    /* Timer is auto-started by SysConfig (timerStartTimer = true).
     * Just clear any stale interrupt and enable NVIC. */
    DL_TimerG_getPendingInterrupt(TIMER_0_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    return true;
}

void BSP_Timer_RegisterCallback(bsp_timer_callback_t cb)
{
    s_callback = cb;
}

/*
 * 10ms periodic timer ISR.
 * Convention: only clear flag and invoke callback.
 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMERG_IIDX_ZERO:
        DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMERG_IIDX_ZERO);
        if (s_callback) {
            s_callback();
        }
        break;
    default:
        break;
    }
}

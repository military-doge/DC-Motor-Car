#include "bsp_encoder.h"
#include "ti_msp_dl_config.h"

/* Volatile encoder counts: written in ISR, read by app layer at 10ms intervals */
static volatile int16_t s_count_a;
static volatile int16_t s_count_b;

bool BSP_Encoder_Init(void)
{
    s_count_a = 0;
    s_count_b = 0;

    /* Clear any stale pending interrupts and enable encoder interrupts */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    return true;
}

int16_t BSP_Encoder_GetCountA(void)
{
    return s_count_a;
}

int16_t BSP_Encoder_GetCountB(void)
{
    return s_count_b;
}

void BSP_Encoder_ResetCounts(void)
{
    s_count_a = 0;
    s_count_b = 0;
}

/*
 * GROUP1_IRQHandler — shared GPIO interrupt for both encoder ports.
 * Implements 2x quadrature decoding on both encoders A and B.
 *
 * NOTE: This ISR does NOT use the callback pattern. Quadrature decoding
 * must process every edge in real-time within the ISR to avoid missing
 * encoder pulses. The decoded counts are exposed via GetCount/ResetCount
 * APIs instead of a callback, since the app layer only needs to read
 * accumulated counts at its own sampling rate (10ms).
 */
void GROUP1_IRQHandler(void)
{
    uint32_t status_a, status_b;

    /* Read and immediately clear pending interrupt status.
     * Clearing first ensures that any new edge arriving during
     * processing will latch a new interrupt — no lost pulses. */
    status_a = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    status_b = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, status_a);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, status_b);

    /* Encoder A: 2x quadrature decoding (if-if handles both edges
     * when they fire simultaneously, not else-if which drops one) */
    if (status_a & ENCODERA_E1A_PIN) {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN)) {
            s_count_a--;
        } else {
            s_count_a++;
        }
    }
    if (status_a & ENCODERA_E1B_PIN) {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN)) {
            s_count_a++;
        } else {
            s_count_a--;
        }
    }

    /* Encoder B: 2x quadrature decoding */
    if (status_b & ENCODERB_E2A_PIN) {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN)) {
            s_count_b--;
        } else {
            s_count_b++;
        }
    }
    if (status_b & ENCODERB_E2B_PIN) {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN)) {
            s_count_b++;
        } else {
            s_count_b--;
        }
    }
}

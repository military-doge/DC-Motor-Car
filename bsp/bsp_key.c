
#include "bsp_key.h"
#include "ti_msp_dl_config.h"

static bsp_key_callback_t s_click_callback = NULL;

bool BSP_Key_Init(void)
{
    /* Match demo: DL_GPIO_initDigitalInput only (NO pull-up in demo) */
    DL_GPIO_initDigitalInput(KEY_key_IOMUX);
    return true;
}

void BSP_Key_RegisterClickCallback(bsp_key_callback_t cb)
{
    s_click_callback = cb;
}

/*
 * Direct port of the demo's working click_N_Double function.
 * Called every 10ms from timer ISR (same as demo).
 *
 * time = 50 means ~500ms timeout for double-click detection.
 * Returns 1 on single click, 2 on double click.
 * Active-low: KEY_STATE == 0 when pressed, >0 when released.
 */
#define KEY_STATE  DL_GPIO_readPins(KEY_PORT, KEY_key_PIN)

static uint8_t click_N_Double(uint8_t time)
{
    static uint8_t  flag_key, count_key, double_key;
    static uint16_t count_single, Forever_count;

    if (KEY_STATE > 0) {
        Forever_count++;
    } else {
        Forever_count = 0;
    }

    if ((KEY_STATE > 0) && (0 == flag_key)) {
        flag_key = 1;
    }

    if (0 == count_key) {
        if (flag_key == 1) {
            double_key++;
            count_key = 1;
        }
        if (double_key == 3) {
            double_key   = 0;
            count_single = 0;
            return 2;   /* double click */
        }
    }

    if (0 == KEY_STATE) {
        flag_key  = 0;
        count_key = 0;
    }

    if (1 == double_key) {
        count_single++;
        if (count_single > time && Forever_count < time) {
            double_key   = 0;
            count_single = 0;
            return 1;   /* single click */
        }
        if (Forever_count > time) {
            double_key   = 0;
            count_single = 0;
        }
    }

    return 0;
}

void BSP_Key_Scan(void)
{
    uint8_t result = click_N_Double(50);

    if (result == 1 && s_click_callback) {
        s_click_callback();
    }
}

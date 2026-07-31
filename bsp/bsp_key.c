#include "bsp_key.h"
#include "ti_msp_dl_config.h"

static bsp_key_callback_t s_click_callback = NULL;
static bsp_key_callback_t s_double_click_callback = NULL;

bool BSP_Key_Init(void)
{
    DL_GPIO_initDigitalInput(KEY_key_IOMUX);
    return true;
}

void BSP_Key_RegisterClickCallback(bsp_key_callback_t cb)
{
    s_click_callback = cb;
}

void BSP_Key_RegisterDoubleClickCallback(bsp_key_callback_t cb)
{
    s_double_click_callback = cb;
}

/*
 * Click / double-click detection state machine.
 * Must be called precisely every 10ms for correct timing.
 * time: double-click wait threshold in ticks (50 = 500ms).
 * Returns: 0 = no action, 1 = single click, 2 = double click.
 * Active-low: KEY_STATE == 0 when pressed, >0 when released.
 */
#define KEY_STATE  DL_GPIO_readPins(KEY_PORT, KEY_key_PIN)

static uint8_t key_click_n_double(uint8_t time)
{
    static uint8_t  s_flag_key, s_count_key, s_double_key;
    static uint16_t s_count_single, s_forever_count;

    if (KEY_STATE > 0) {
        s_forever_count++;
    } else {
        s_forever_count = 0;
    }

    if ((KEY_STATE > 0) && (s_flag_key == 0)) {
        s_flag_key = 1;
    }

    if (s_count_key == 0) {
        if (s_flag_key == 1) {
            s_double_key++;
            s_count_key = 1;
        }
        if (s_double_key == 3) {
            s_double_key   = 0;
            s_count_single = 0;
            return 2;   /* double click */
        }
    }

    if (KEY_STATE == 0) {
        s_flag_key  = 0;
        s_count_key = 0;
    }

    if (s_double_key == 1) {
        s_count_single++;
        if (s_count_single > time && s_forever_count < time) {
            s_double_key   = 0;
            s_count_single = 0;
            return 1;   /* single click */
        }
        if (s_forever_count > time) {
            s_double_key   = 0;
            s_count_single = 0;
        }
    }

    return 0;
}

void BSP_Key_Scan(void)
{
    uint8_t result = key_click_n_double(50);

    if (result == 1 && s_click_callback) {
        s_click_callback();
    }
    if (result == 2 && s_double_click_callback) {
        s_double_click_callback();
    }
}

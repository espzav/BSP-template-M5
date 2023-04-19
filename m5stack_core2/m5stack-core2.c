/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "bsp/esp-box.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lvgl_port.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "ESP-BOX";

/** @cond */
_Static_assert(CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0, "Touch buttons must be supported for this BSP");
/** @endcond */

static lv_disp_t *disp;
static lv_indev_t *disp_indev = NULL;
sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static esp_lcd_touch_handle_t tp;   // LCD touch handle
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;

esp_err_t bsp_i2c_init(void)
{
    static bool i2c_initialized = false;

    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_I2C_NUM));
    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_sdcard_mount(void)
{
    //TODO: Continue here with SD Card mount. Please inspire in Korvo-2 implementation
}

esp_err_t bsp_sdcard_unmount(void)
{
    //TODO: Continue here with SD Card unmount. Please inspire in Korvo-2 implementation
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    //TODO: Continue here with I2S initialization. Please inspire in ESP-BOX implementation

    return ESP_OK;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    //TODO: Continue here with poweramp enable. Please inspire in ESP-BOX/Korvo-2 implementation
    //Use AXP192 IO component (or make it)

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    //NS4168
    //TODO: Continue here with speaker codec initialization. Use esp_codec_dev. Please inspire in ESP-BOX implementation
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    //SPM1423
    //TODO: Continue here with microphone codec initialization. Use esp_codec_dev. Please inspire in ESP-BOX implementation
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

static esp_err_t bsp_display_brightness_init(void)
{
    //TODO: Continue here with brightness initialization. Please inspire in Korvo-2 implementation
    //Use AXP192 IO component (or make it)
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    //TODO: Continue here with brightness set. Please inspire in Korvo-2 implementation
    //Use AXP192 IO component (or make it)
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    //ILI9342C
    //TODO: Continue here with LCD initialization. Please inspire in Korvo-2 implementation
    //Use AXP192 IO component for RST and PWR (or make it)
}

static lv_disp_t *bsp_display_lcd_init(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    //FT6336U (compatible with FT5x06)
    //TODO: Continue here with LCD Touch initialization. Please inspire in Korvo-2 implementation
}

static lv_indev_t *bsp_display_indev_init(lv_disp_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_disp_t *bsp_display_start(void)
{
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&lvgl_cfg));

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

esp_err_t bsp_leds_init(void)
{
    //TODO: Continue here with led init. Please inspire in Korvo-2 implementation
}

esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on)
{
    //TODO: Continue here with led control. Please inspire in Korvo-2 implementation
}

esp_err_t bsp_button_init(const bsp_button_t btn)
{
    //TODO: Continue here with three buttons init. Please inspire in Korvo-2 implementation (get it from ESP LCD touch)
}

bool bsp_button_get(const bsp_button_t btn)
{
    //TODO: Continue here with three buttons get. Please inspire in Korvo-2 implementation (get it from ESP LCD touch)
}

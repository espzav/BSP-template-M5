# ESP BSP template M5Stack

This repository is a template for making ESP-BSP packages. It contains example for each part of BSP and one complex example for use all BSP parts.

## Example of implementation some components in BSPs

* I2C (esp-box)
* SPI LCD (esp-box)
* RGB LCD (esp32_s3_lcd_ev_board)
* I2C LCD Touch (esp-box)
* SPIFFS (esp-box)
* SD Card (esp32_s3_korvo_2)
* Buttons (esp32_s3_korvo_2, esp-box-lite)
* LEDs (esp32_s3_korvo_2)
* Audio (esp-box, esp-box-lite)
* Camera (esp32_s3_korvo_2)
* Battery (esp32_s3_korvo_2)
* USB (esp32_s3_usb_otg)

# Step-by-step guide

|BSP API: | [I2C](#i2c) | [LCD display](#lcd_display) | [LCD Touch](#lcd_touch) | [SPIFFS](#spiffs) | [SD Card](#sd_card) | [Buttons](#buttons) | [LEDs](#leds) | [Audio](#audio) | [Camera](#camera) | [Battery](#battery) | [USB](#usb) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |

## 1. Create folder

Create folder with name of the board in the root of [ESP-BSP](https://github.com/espressif/esp-bsp) repository.

## 2. Copy template folder

Copy all files and folders from template into new folder.

## 3. Update file idf_component.yml

Edit file `idf_component.yml` in BSP folder:
* Put version to 1.0.0 (or lower for beta version)
* Edit description, url, target and dependencies

## 4. Update Kconfig

Edit file `Kconfig` with settings some components. You can copy some settings from another BSPs.

## 4. Update Readme file

Edit file `README.md` in BSP folder:
* Edit header
* Put link to HW reference or another page about the board
* Put image of the board
* Write short description about the board and the features of this board

## 5. Rename files

Rename main *.c file and header file of this BSP. 
* Rename `m5stack-core2.c` and `include\bsp\m5stack-core2.h`
* Update `include\bsp\esp-bsp.h` with renamed header file
* Update `CMakeLists.txt` 

<a id="i2c"></a>
## 6. I2C (mandatory)

Update constants defined in BSP header - settings:
* `BSP_I2C_NUM` (value: `0`/`1` - depends on ESP chip)

Update constants defined in BSP header - I2C pins:
* `BSP_I2C_SCL` (value: `GPIO_NUM_x`)
* `BSP_I2C_SDA` (value: `GPIO_NUM_x`) 

Functions, which **must** be defined in BSP `*.c` file:
* `esp_err_t bsp_i2c_init(void);`
* `esp_err_t bsp_i2c_deinit(void);`

**Note:** When I2C is needed in some component, it should be initialized inside the component (call `bsp_i2c_init`). The function `bsp_i2c_init` should initialize I2C only once.

<a id="lcd_display"></a>
## 7. Display (mandatory)

Display functions are divided into two parts. The first part is only HW initialization (`include\bsp\display.h`) and second part is initialization of the LVG (BSP header). 

Update display constants defined in `display.h`:
* `BSP_LCD_COLOR_FORMAT`    (values: `ESP_LCD_COLOR_FORMAT_RGB565`/`ESP_LCD_COLOR_FORMAT_RGB888`)
* `BSP_LCD_BIGENDIAN`       (values: `0`/`1`)
* `BSP_LCD_BITS_PER_PIXEL`  (values: `16`/`18`)
* `BSP_LCD_COLOR_SPACE`     (values: `ESP_LCD_COLOR_SPACE_RGB`/`ESP_LCD_COLOR_SPACE_BGR`)

Update display constants defined in BSP header - pins:
* `BSP_LCD_DATA0`       (value: `GPIO_NUM_x`)
* `BSP_LCD_PCLK`        (value: `GPIO_NUM_x`) 
* `BSP_LCD_CS`          (value: `GPIO_NUM_x`)
* `BSP_LCD_DC`          (value: `GPIO_NUM_x`)
* `BSP_LCD_RST`         (value: `GPIO_NUM_x`)
* `BSP_LCD_BACKLIGHT`   (value: `GPIO_NUM_x`)

Update display constants defined in BSP header - settings:
* `BSP_LCD_H_RES`               (value: integer)
* `BSP_LCD_V_RES`               (value: integer)
* `BSP_LCD_PIXEL_CLOCK_HZ`      (value: integer)
* `BSP_LCD_SPI_NUM`             (value: `0`/`1` - depends on ESP chip)
* `BSP_LCD_DRAW_BUFF_SIZE`      (value `BSP_LCD_H_RES * x`)
* `BSP_LCD_DRAW_BUFF_DOUBLE`    (value: `0`/`1`)

Functions, which **must** be defined in BSP `*.c` file:
* `esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io);`
* `lv_disp_t *bsp_display_start(void);`
* `bool bsp_display_lock(uint32_t timeout_ms);`
* `void bsp_display_unlock(void);`
* `esp_err_t bsp_display_brightness_set(int brightness_percent);`
* `esp_err_t bsp_display_backlight_on(void);`
* `esp_err_t bsp_display_backlight_off(void);`
* `void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation);`

<a id="lcd_touch"></a>
## 8. LCD Touch (mandatory)

LCD touch functions are divided into two parts. The first part is only HW initialization (`include\bsp\touch.h`) and second part is initialization of the LVG (BSP header).

Functions, which **must** be defined in BSP `*.c` file:
* `esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);`
* `lv_indev_t *bsp_display_get_input_dev(void);`

<a id="spiffs"></a>
## 9. SPI Flash File System (mandatory)

The SPIFFS is supported by all ESP chips internally. These functions are same for all BSPs.

Update SPIFFS constants defined in BSP header - settings:
* `BSP_MOUNT_POINT` (value: string)

Functions, which **must** be defined in BSP `*.c` file:
* `esp_err_t bsp_spiffs_mount(void);`
* `esp_err_t bsp_spiffs_unmount(void);`

<a id="sd_card"></a>
## 10. SD Card (optional)

Update SD card constants defined in BSP header - pins:
* `BSP_SD_CLK` (value: `GPIO_NUM_x`)
* `BSP_SD_CMD` (value: `GPIO_NUM_x`) 
* `BSP_SD_D0`  (value: `GPIO_NUM_x`) 

Update SD card constants defined in BSP header - settings:
* `BSP_MOUNT_POINT` (value: string)

Functions, which **should** be defined in BSP `*.c` file:
* `esp_err_t bsp_sdcard_mount(void);`
* `esp_err_t bsp_sdcard_unmount(void);`

and put the SD card handler into BSP header file as extern:
```c
extern sdmmc_card_t *bsp_sdcard;
```

<a id="buttons"></a>
## 11. Buttons (optional)

Buttons can be connected as GPIO pins, inside [button component](https://components.espressif.com/components/espressif/button), from LCD touch or IO expander. Recommended solution is using the button component for all buttons (GPIO, ADC, etc...). 

Add buttons enum into BSP header:
```c
typedef enum {
    BSP_BUTTON_MAIN = 0,
    BSP_BUTTON_xxx,
    ...
    BSP_BUTTON_NUM
} bsp_button_t;
```

If component buttons is used, create configuration structure in BSP *.c file:
```c
const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_MAIN,
        .adc_button_config.min = 2310, // middle is 2410mV
        .adc_button_config.max = 2510
    },
    ...
};
```

and put the configuration into BSP header file as extern:
```c
extern const button_config_t bsp_button_config[BSP_BUTTON_NUM];
```

Functions, which **should** be defined in BSP `*.c` file (when not used button component):
* `esp_err_t bsp_button_init(const bsp_button_t btn);`
* `bool bsp_button_get(const bsp_button_t btn);`

<a id="leds"></a>
## 12. LEDs (optional)

Leds are usually controlled as GPIO or it can be controlled by IO expander.

Add LEDs enum into BSP header:
```c
typedef enum bsp_led_t {
    BSP_LED_BLUE,
    BSP_LED_RED,
} bsp_led_t;
```

Functions, which **should** be defined in BSP `*.c` file:
* `esp_err_t bsp_leds_init(void);`
* `esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on);`

<a id="audio"></a>
## 13. Audio (optional)

BSP audio functions are divided into two parts. The first part is initialization of the I2S (and I2C) for communication with codec chips and second part is initialization of the codec chips (for speaker and for microphone). For codec chips is usually used `esp_codec_dev` component. 

Update audio (I2S) constants defined in BSP header - pins:
* `BSP_I2S_SCLK` (value: `GPIO_NUM_x`)
* `BSP_I2S_MCLK` (value: `GPIO_NUM_x`) 
* `BSP_I2S_LCLK` (value: `GPIO_NUM_x`) 
* `BSP_I2S_DOUT` (value: `GPIO_NUM_x`) 
* `BSP_I2S_DSIN` (value: `GPIO_NUM_x`) 
* `BSP_POWER_AMP_IO`  (value: `GPIO_NUM_x`) 

Update audio configuration constants defined in BSP header - settings:
```c
#define BSP_I2S_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S_MCLK,  \
        .bclk = BSP_I2S_SCLK,  \
        .ws = BSP_I2S_LCLK,    \
        .dout = BSP_I2S_DOUT,  \
        .din = BSP_I2S_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
{                                                                                                 \
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
    .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
    .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
}
```

Functions, which **should** be defined in BSP `*.c` file:
* `esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel);`
* `esp_err_t bsp_audio_poweramp_enable(const bool enable);`
* `esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)`
* `esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);`

<a id="camera"></a>
## 14. Camera (optional)

Update camera constants defined in BSP header - pins:
* `BSP_CAMERA_XCLK` (value: `GPIO_NUM_x`)
* `BSP_CAMERA_PCLK` (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_VSYNC`(value: `GPIO_NUM_x`) 
* `BSP_CAMERA_HSYNC`(value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D0`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D1`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D2`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D3`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D4`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D5`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D6`   (value: `GPIO_NUM_x`) 
* `BSP_CAMERA_D7`   (value: `GPIO_NUM_x`) 

Update camera configuration constants defined in BSP header - settings:
```c
#define BSP_CAMERA_DEFAULT_CONFIG         \
    {                                     \
        .pin_pwdn = GPIO_NUM_NC,          \
        .pin_reset = GPIO_NUM_NC,         \
        .pin_xclk = BSP_CAMERA_XCLK,      \
        .pin_sccb_sda = GPIO_NUM_NC,      \
        .pin_sccb_scl = GPIO_NUM_NC,      \
        .pin_d7 = BSP_CAMERA_D7,          \
        .pin_d6 = BSP_CAMERA_D6,          \
        .pin_d5 = BSP_CAMERA_D5,          \
        .pin_d4 = BSP_CAMERA_D4,          \
        .pin_d3 = BSP_CAMERA_D3,          \
        .pin_d2 = BSP_CAMERA_D2,          \
        .pin_d1 = BSP_CAMERA_D1,          \
        .pin_d0 = BSP_CAMERA_D0,          \
        .pin_vsync = BSP_CAMERA_VSYNC,    \
        .pin_href = BSP_CAMERA_HSYNC,     \
        .pin_pclk = BSP_CAMERA_PCLK,      \
        .xclk_freq_hz = 16000000,         \
        .ledc_timer = LEDC_TIMER_0,       \
        .ledc_channel = LEDC_CHANNEL_0,   \
        .pixel_format = PIXFORMAT_RGB565, \
        .frame_size = FRAMESIZE_240X240,  \
        .jpeg_quality = 12,               \
        .fb_count = 2,                    \
        .fb_location = CAMERA_FB_IN_PSRAM,\
        .sccb_i2c_port = BSP_I2C_NUM,     \
    }
```

<a id="battery"></a>
## 15. Battery (optional)

When the board is using ADC for measuring voltage of the batery, there should be added API inside BSP.

Update constants defined in BSP header:
* `BSP_BATTERY_VOLTAGE`     (value: `GPIO_NUM_x`)
* `BSP_BATTERY_VOLTAGE_DIV` (value: integer)

Functions, which **should** be defined in BSP `*.c` file:
* `esp_err_t bsp_voltage_init(void);`
* `int bsp_voltage_battery_get(void);`

<a id="usb"></a>
## 16. USB

If the board support USB connectiion, there should be defined USB pins.

Update USB constants defined in BSP header - USB pins:
* `BSP_USB_POS` (value: `USBPHY_DP_NUM`)
* `BSP_USB_NEG` (value: `USBPHY_DM_NUM`) 

## SquareLine Template

After the BSP is done, it should be added into SquareLine tepmplates in folder `esp-bsp\SquareLine`. Follow [this](https://github.com/espressif/esp-bsp/blob/master/SquareLine/README.md) guide for add BSP into SquareLine as a template.
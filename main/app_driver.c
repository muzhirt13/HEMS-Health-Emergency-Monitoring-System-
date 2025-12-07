#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc_oneshot.h"
#include "driver/i2c.h"

/* ---- U8G2 ---- */
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

/* ---- RainMaker ---- */
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_standard_devices.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_utils.h"

#include "app_wifi.h"
#include "app_reset.h"

static const char *TAG = "health_monitor";

/* ==== PIN DEFINITIONS (ADJUST IF NEEDED) ==== */
#define I2C_SDA_PIN         6
#define I2C_SCL_PIN         7

#define GPIO_TEMP_MCP9700   2     // ADC1_CH2
#define GPIO_PULSE_KS0015   3     // ADC1_CH3

#define GPIO_BUZZER         4
#define GPIO_FAILSAFE_BTN   5     // active LOW, internal pull-up

/* ==== THRESHOLDS ==== */
#define FALL_ACCEL_THRESHOLD_G      2.5f   // example threshold
#define HIGH_BPM_THRESHOLD          120
#define HIGH_TEMP_THRESHOLD_C       35.0f

/* ==== GLOBAL FLAGS (from RainMaker) ==== */
static bool g_fall_detector_enabled  = true;
static bool g_pulse_monitor_enabled  = true;
static bool g_temp_monitor_enabled   = true;
static bool g_buzzer_muted           = false;

/* ==== HANDLES ==== */
static u8g2_t u8g2;
static adc_oneshot_unit_handle_t adc_handle;
static esp_rmaker_node_t *g_node = NULL;
static esp_rmaker_device_t *g_fall_device = NULL;
static esp_rmaker_device_t *g_pulse_device = NULL;
static esp_rmaker_device_t *g_temp_device = NULL;
static esp_rmaker_device_t *g_buzzer_device = NULL;

/* ---- forward declarations ---- */
static void buzzer_set(bool on);
static void display_message(const char *line1, const char *line2);
static float read_temperature_c(void);
static int   read_bpm(void);
static bool  detect_fall(void);

/* ============================================================
 *  OLED HELPER
 * ============================================================ */
static void init_display(void)
{
    u8g2_esp32_hal_t u8g2_hal_config = U8G2_ESP32_HAL_DEFAULT;
    u8g2_hal_config.sda = I2C_SDA_PIN;
    u8g2_hal_config.scl = I2C_SCL_PIN;
    u8g2_esp32_hal_init(u8g2_hal_config);

    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );
    u8g2_SetI2CAddress(&u8g2, 0x3C << 1);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // wake up
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);

    display_message("Health Monitor", "Waiting...");
}

static void display_message(const char *line1, const char *line2)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 12, line1);
    if (line2) {
        u8g2_DrawStr(&u8g2, 0, 28, line2);
    }
    u8g2_SendBuffer(&u8g2);
}

/* ============================================================
 *  BUZZER + BUTTON
 * ============================================================ */
static void buzzer_set(bool on)
{
    if (g_buzzer_muted) {
        gpio_set_level(GPIO_BUZZER, 0);
        return;
    }
    gpio_set_level(GPIO_BUZZER, on ? 1 : 0);
}

static void init_buzzer_button(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_BUZZER) | (1ULL << GPIO_FAILSAFE_BTN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    // configure buzzer as output first
    io_conf.pin_bit_mask = (1ULL << GPIO_BUZZER);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_BUZZER, 0);

    // configure button as input with pull-up
    io_conf.pin_bit_mask = (1ULL << GPIO_FAILSAFE_BTN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

/* ============================================================
 *  ADC + SENSORS (MCP9700 + KS0015)
 * ============================================================ */
static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,  // up to ~3.6V
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_cfg)); // GPIO2
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &chan_cfg)); // GPIO3
}

static float read_temperature_c(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &raw) != ESP_OK) {
        return 25.0f;
    }

    // VERY rough estimate: assume Vref = 1100mV and 12-bit ADC
    float v_mV = (raw / 4095.0f) * 3300.0f;  // if VDD=3.3V
    // MCP9700: Vout = 500mV + 10mV/°C * T
    float temp_c = (v_mV - 500.0f) / 10.0f;
    return temp_c;
}

/* 
 * VERY simplified BPM estimator stub.
 * You should replace this with a proper KS0015 lib or 
 * algorithm (peak detection over window).
 */
static int read_bpm(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &raw) != ESP_OK) {
        return 70;
    }

    // Dummy mapping just so logic works; replace with real BPM logic
    int bpm = 60 + (raw % 80);   // 60–140
    return bpm;
}

/* ============================================================
 *  MPU6050 FALL DETECTION (STUB)
 * ============================================================ */
static void init_mpu6050(void)
{
    // TODO: init I2C + MPU6050 properly.
    // For now, assume you have driver functions elsewhere.
}

static bool detect_fall(void)
{
    // TODO: read accel X/Y/Z from MPU6050 via I2C.
    // For now, this is a stub that always returns false.
    // Example logic (pseudo):
    //
    // float ax, ay, az;
    // mpu6050_read_accel_g(&ax, &ay, &az);
    // float mag = sqrtf(ax*ax + ay*ay + az*az);
    // if (mag > FALL_ACCEL_THRESHOLD_G) return true;
    //
    // return false;

    return false;
}

/* ============================================================
 *  RAINMAKER WRITE CALLBACK
 * ============================================================ */
static esp_err_t health_write_cb(const esp_rmaker_device_t *device,
                                 const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val,
                                 void *priv)
{
    const char *dev_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);

    ESP_LOGI(TAG, "Write to device: %s, param: %s", dev_name, param_name);

    if (strcmp(dev_name, "Fall Detector") == 0 &&
        strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        g_fall_detector_enabled = val.val.b;
        esp_rmaker_param_update_and_report(param, val);
    } else if (strcmp(dev_name, "Pulse Monitor") == 0 &&
               strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        g_pulse_monitor_enabled = val.val.b;
        esp_rmaker_param_update_and_report(param, val);
    } else if (strcmp(dev_name, "Temp Monitor") == 0 &&
               strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        g_temp_monitor_enabled = val.val.b;
        esp_rmaker_param_update_and_report(param, val);
    } else if (strcmp(dev_name, "Buzzer Mute") == 0 &&
               strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        g_buzzer_muted = val.val.b;
        esp_rmaker_param_update_and_report(param, val);
        if (g_buzzer_muted) {
            buzzer_set(false);
        }
    }

    return ESP_OK;
}

/* ============================================================
 *  SENSOR + EVENT TASK
 * ============================================================ */
static void sensor_task(void *arg)
{
    display_message("Health Monitor", "Running...");

    while (1) {
        bool event_triggered = false;
        char msg1[16] = {0};
        char msg2[16] = {0};

        /* ---- Fail-safe button to mute buzzer ---- */
        if (gpio_get_level(GPIO_FAILSAFE_BTN) == 0) {  // active LOW
            g_buzzer_muted = true;
            buzzer_set(false);
            snprintf(msg1, sizeof(msg1), "BUZZER MUTED");
            snprintf(msg2, sizeof(msg2), "BUTTON PRESS");
            display_message(msg1, msg2);

            // Also reflect in RainMaker
            if (g_buzzer_device) {
                esp_rmaker_param_t *p =
                    esp_rmaker_device_get_param_by_type(g_buzzer_device,
                                                        ESP_RMAKER_PARAM_POWER);
                if (p) {
                    esp_rmaker_param_val_t v = esp_rmaker_bool(true);
                    esp_rmaker_param_update_and_report(p, v);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* ---- Fall detection ---- */
        if (g_fall_detector_enabled && detect_fall()) {
            buzzer_set(true);
            snprintf(msg1, sizeof(msg1), "FALL DETECTED");
            snprintf(msg2, sizeof(msg2), "ALERT!");
            display_message(msg1, msg2);
            event_triggered = true;
        }

        /* ---- Pulse monitoring ---- */
        if (g_pulse_monitor_enabled) {
            int bpm = read_bpm();
            if (!event_triggered && bpm > HIGH_BPM_THRESHOLD) {
                buzzer_set(true);
                snprintf(msg1, sizeof(msg1), "HIGH BPM");
                snprintf(msg2, sizeof(msg2), "BPM:%d", bpm);
                display_message(msg1, msg2);
                event_triggered = true;
            }
        }

        /* ---- Temp monitoring ---- */
        if (g_temp_monitor_enabled) {
            float temp_c = read_temperature_c();
            if (!event_triggered && temp_c >= HIGH_TEMP_THRESHOLD_C) {
                buzzer_set(true);
                snprintf(msg1, sizeof(msg1), "HIGH TEMP!");
                snprintf(msg2, sizeof(msg2), "%.1f C", temp_c);
                display_message(msg1, msg2);
                event_triggered = true;
            }
        }

        /* ---- No event: show normal status ---- */
        if (!event_triggered) {
            buzzer_set(false);
            char line1[16], line2[16];
            float temp_c = read_temperature_c();
            int bpm = read_bpm();
            snprintf(line1, sizeof(line1), "T:%.1fC BPM:%d", temp_c, bpm);
            snprintf(line2, sizeof(line2), "OK");
            display_message(line1, line2);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // adjust as needed
    }
}

/* ============================================================
 *  RAINMAKER SETUP
 * ============================================================ */
static void init_rainmaker(void)
{
    esp_rmaker_config_t cfg = {
        .enable_time_sync = true,
    };

    g_node = esp_rmaker_node_init(&cfg, "HealthMonitor", "Health Monitor Node");
    if (!g_node) {
        ESP_LOGE(TAG, "Failed to init RainMaker node");
        return;
    }

    /* ---- Fall Detector Device ---- */
    g_fall_device = esp_rmaker_device_create("Fall Detector",
                                             ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_param(g_fall_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Fall Detector"));
    esp_rmaker_param_t *fall_power =
        esp_rmaker_param_create(ESP_RMAKER_DEF_POWER_NAME, ESP_RMAKER_PARAM_POWER,
                                esp_rmaker_bool(true),
                                PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(g_fall_device, fall_power);
    esp_rmaker_device_add_cb(g_fall_device, health_write_cb, NULL);
    esp_rmaker_node_add_device(g_node, g_fall_device);

    /* ---- Pulse Monitor Device ---- */
    g_pulse_device = esp_rmaker_device_create("Pulse Monitor",
                                              ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_param(g_pulse_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Pulse Monitor"));
    esp_rmaker_param_t *pulse_power =
        esp_rmaker_param_create(ESP_RMAKER_DEF_POWER_NAME, ESP_RMAKER_PARAM_POWER,
                                esp_rmaker_bool(true),
                                PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(g_pulse_device, pulse_power);
    esp_rmaker_device_add_cb(g_pulse_device, health_write_cb, NULL);
    esp_rmaker_node_add_device(g_node, g_pulse_device);

    /* ---- Temp Monitor Device ---- */
    g_temp_device = esp_rmaker_device_create("Temp Monitor",
                                             ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_param(g_temp_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Temp Monitor"));
    esp_rmaker_param_t *temp_power =
        esp_rmaker_param_create(ESP_RMAKER_DEF_POWER_NAME, ESP_RMAKER_PARAM_POWER,
                                esp_rmaker_bool(true),
                                PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(g_temp_device, temp_power);
    esp_rmaker_device_add_cb(g_temp_device, health_write_cb, NULL);
    esp_rmaker_node_add_device(g_node, g_temp_device);

    /* ---- Buzzer Mute Device ---- */
    g_buzzer_device = esp_rmaker_device_create("Buzzer Mute",
                                               ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_param(g_buzzer_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Buzzer Mute"));
    esp_rmaker_param_t *buzzer_power =
        esp_rmaker_param_create(ESP_RMAKER_DEF_POWER_NAME, ESP_RMAKER_PARAM_POWER,
                                esp_rmaker_bool(false),
                                PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(g_buzzer_device, buzzer_power);
    esp_rmaker_device_add_cb(g_buzzer_device, health_write_cb, NULL);
    esp_rmaker_node_add_device(g_node, g_buzzer_device);

    /* ---- Start RainMaker ---- */
    ESP_ERROR_CHECK(esp_rmaker_start());
}

/* ============================================================
 *  APP MAIN
 * ============================================================ */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Wi-Fi + provisioning (use helper from RainMaker examples) */
    app_wifi_init();
    app_wifi_start(POP_TYPE_RANDOM);  // or POP_TYPE_NONE after first provisioning

    /* OLED + Sensors + GPIO */
    init_display();
    init_buzzer_button();
    init_adc();
    init_mpu6050();

    /* RainMaker (voice assistants will see the devices as standard switches) */
    init_rainmaker();

    /* Create task for periodic sensor monitoring */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    /* Optional: reset button for factory reset / Wi-Fi reset */
    app_reset_button_create(GPIO_NUM_9, true); // adjust pin if you want
}

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// U8G2 + HAL
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

// RainMaker
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_utils.h"
#include "app_wifi.h"

static const char *TAG = "health_monitor";

/* ---------- Pin configuration (CHANGE IF NEEDED) ---------- */
// I2C (Xiao ESP32-C3 typical mapping: SDA=D4, SCL=D5)
#define I2C_PORT_NUM          I2C_NUM_0
#define I2C_SDA_GPIO          6
#define I2C_SCL_GPIO          7
#define I2C_FREQ_HZ           400000

// OLED (SSD1306 128x32)
#define OLED_I2C_ADDR_7BIT    0x3C

// MPU6050
#define MPU6050_I2C_ADDR      0x68
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// ADC for MCP9700 and KS0015 (ESP32-C3: GPIO0..4 -> ADC1 channels 0..4)
#define TEMP_ADC_CHANNEL      ADC_CHANNEL_2   // GPIO2
#define PULSE_ADC_CHANNEL     ADC_CHANNEL_3   // GPIO3

// Buzzer + local mute button
#define BUZZER_GPIO           10
#define BUTTON_MUTE_GPIO      9   // Active LOW with pull-up

// Sensor thresholds
#define FALL_G_THRESHOLD      2.0f    // overall magnitude |a| > 2g
#define FALL_Y_THRESHOLD      1.0f    // |ay| > 1g specifically on Y
#define HIGH_BPM_THRESHOLD    90.0f
#define HIGH_TEMP_THRESHOLD_C 39.0f

// Task timing
#define SENSOR_TASK_DELAY_MS  200

/* ---------- Globals ---------- */

// OLED
static u8g2_t u8g2;

// ADC
static adc_oneshot_unit_handle_t adc_handle = NULL;
static bool adc_cali_enabled = false;
static adc_cali_handle_t adc_cali_handle = NULL;

// RainMaker devices and params
static esp_rmaker_device_t *fall_device   = NULL;
static esp_rmaker_device_t *pulse_device  = NULL;
static esp_rmaker_device_t *temp_device   = NULL;
static esp_rmaker_device_t *buzzer_device = NULL;

static esp_rmaker_param_t *buzzer_power_param = NULL;

// NEW: live value params for app
static esp_rmaker_param_t *pulse_value_param = NULL;
static esp_rmaker_param_t *temp_value_param  = NULL;

// Local states controlled by RainMaker
static bool fall_enabled   = false;
static bool pulse_enabled  = false;
static bool temp_enabled   = false;
static bool buzzer_muted   = false;  // "help found" / mute buzzer

// Latched fall flag – stays true after fall until cleared
static bool fall_latched = false;

/* ---------- I2C helpers ---------- */

static esp_err_t i2c_write_byte(uint8_t dev_addr_7bit, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_7bit << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ---------- MPU6050 (simple accel for fall detection) ---------- */

static esp_err_t mpu6050_init(void)
{
    // Wake up device (clear sleep bit)
    esp_err_t ret = i2c_write_byte(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: 0x%x", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return ret;
}

static esp_err_t mpu6050_read_accel(float *ax_g, float *ay_g, float *az_g)
{
    uint8_t buf[6];
    esp_err_t ret = i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);

    // Default ±2g => 16384 LSB/g
    const float scale = 1.0f / 16384.0f;
    *ax_g = raw_ax * scale;
    *ay_g = raw_ay * scale;
    *az_g = raw_az * scale;
    return ESP_OK;
}

/* ---------- ADC: MCP9700 + KS0015 ---------- */

static bool adc_calibration_init(adc_unit_t unit, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &handle) == ESP_OK) {
        *out_handle = handle;
        return true;
    }
    *out_handle = NULL;
    return false;
}

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,    // up to ~3.3 V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TEMP_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, PULSE_ADC_CHANNEL, &chan_cfg));

    adc_cali_enabled = adc_calibration_init(ADC_UNIT_1, &adc_cali_handle);
}

static float adc_raw_to_voltage(int raw)
{
    int mv = 0;
    if (adc_cali_enabled && adc_cali_handle) {
        if (adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) != ESP_OK) {
            mv = 0;
        }
    } else {
        // Approximate 0-3.3V range, 12-bit
        mv = (int)((raw / 4095.0f) * 3300.0f);
    }
    return (float)mv / 1000.0f;
}

// MCP9700: 0.5 V at 0°C, 10 mV/°C => T = (V - 0.5) / 0.01
static float read_temperature_c(void)
{
    int raw = 0;
    if (adc_handle == NULL) return 0.0f;
    if (adc_oneshot_read(adc_handle, TEMP_ADC_CHANNEL, &raw) != ESP_OK) return 0.0f;

    float v = adc_raw_to_voltage(raw);
    float t = (v - 0.5f) / 0.01f;
    return t;
}

// KS0015 is an analog IR pulse sensor; proper BPM would need filtering/timing.
// For now, we just compute a "pseudo-BPM" from signal magnitude and threshold it.
static float read_pulse_bpm_simplified(void)
{
    int raw = 0;
    if (adc_handle == NULL) return 0.0f;

    esp_err_t err = adc_oneshot_read(adc_handle, PULSE_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Pulse ADC read failed: %s", esp_err_to_name(err));
        return 0.0f;
    }

    float v = adc_raw_to_voltage(raw);   // in Volts

    // Basic sanity checks
    // If the pin is basically at 0V or at full scale, assume "no valid reading".
    if (raw < 50 || raw > 4000) {
        ESP_LOGW(TAG, "Pulse signal out of range: raw=%d (V=%.3f)", raw, v);
        return 0.0f;   // treat as no heartbeat
    }

    // Rough mapping just to get a usable number
    float bpm = (raw / 4095.0f) * 200.0f;
    return bpm;
}

/* ---------- OLED (u8g2) ---------- */

static void oled_init(void)
{
    u8g2_esp32_hal_t hal = U8G2_ESP32_HAL_DEFAULT;
    hal.bus.i2c.sda = I2C_SDA_GPIO;
    hal.bus.i2c.scl = I2C_SCL_GPIO;
    u8g2_esp32_hal_init(hal);

    // 128x32 SSD1306 I2C
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        // Use the HAL callbacks from u8g2-hal-esp-idf
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);

    // I2C address: 0x3C for SSD1306 -> shift left by 1 for u8x8
    u8x8_SetI2CAddress(&u8g2.u8x8, OLED_I2C_ADDR_7BIT << 1);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
}

/* ---------- Buzzer + button ---------- */

static void buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUZZER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, 0);
}

static void buzzer_set(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? 1 : 0);
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_MUTE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static bool button_mute_pressed(void)
{
    // Active LOW
    int level = gpio_get_level(BUTTON_MUTE_GPIO);
    return (level == 0);
}

/* ---------- RainMaker write callback ---------- */

static esp_err_t switch_write_cb(const esp_rmaker_device_t *device,
                                 const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val,
                                 void *priv_data,
                                 esp_rmaker_write_ctx_t *ctx)
{
    const char *dev_name   = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);

    ESP_LOGI(TAG, "RM write_cb: device=%s param=%s val=%s",
             dev_name, param_name, val.val.b ? "true" : "false");

    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        bool v = val.val.b;

        if (strcmp(dev_name, "Fall Detector") == 0) {
            fall_enabled = v;
        } else if (strcmp(dev_name, "Pulse Monitor") == 0) {
            pulse_enabled = v;
        } else if (strcmp(dev_name, "Temp Monitor") == 0) {
            temp_enabled = v;
        } else if (strcmp(dev_name, "Buzzer Mute") == 0) {
            buzzer_muted = v;
            if (buzzer_muted) {
                // When user mutes from app, stop buzzer and clear fall latch
                buzzer_set(false);
                fall_latched = false;
            }
        }

        // Report back to cloud
        esp_rmaker_param_update_and_report(param, val);
    }
    return ESP_OK;
}

/* ---------- OLED helper ---------- */

static void oled_draw_status(const char *line1, const char *line2)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 10, line1);
    if (line2) {
        u8g2_DrawStr(&u8g2, 0, 24, line2);
    }
    u8g2_SendBuffer(&u8g2);
}

/* ---------- Hardware driver init (local) ---------- */

static void app_driver_init(void)
{
    // DO NOT install I2C driver here.
    // u8g2_esp32_hal_init() inside oled_init() will set up I2C for us.

    oled_init();                           // This calls u8g2_esp32_hal_init(...)
    ESP_ERROR_CHECK(mpu6050_init());       // Now we can safely use I2C on the same bus
    adc_init();
    buzzer_init();
    button_init();

    oled_draw_status("Health Monitor", "Waiting for WiFi");
}

/* ---------- RainMaker event helpers ---------- */

static void send_event_fall_detected(void)
{
    esp_rmaker_raise_alert("Fall detected! Check patient immediately.");
}

static void send_event_high_bpm(float bpm)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "High BPM: %.1f detected.", bpm);
    esp_rmaker_raise_alert(msg);
}

static void send_event_high_temp(float temp_c)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "High temperature: %.1f C detected.", temp_c);
    esp_rmaker_raise_alert(msg);
}

static void send_event_false_alarm(void)
{
    esp_rmaker_raise_alert("Alert cleared. False alarm / help found.");
}

/* ---------- Sensor + logic task ---------- */

static void sensor_task(void *arg)
{
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float bpm = 0.0f;
    float temp_c = 0.0f;

    // For edge-detected notifications (prevent spam)
    static bool last_alert_fall  = false;
    static bool last_alert_pulse = false;
    static bool last_alert_temp  = false;

    while (1) {
        bool alert_fall  = false;
        bool alert_pulse = false;
        bool alert_temp  = false;

        bool any_alert = false;

        char line1[32] = {0};
        char line2[32] = {0};

        // -------- Local fail-safe buzzer mute button --------
        if (button_mute_pressed()) {
            if (!buzzer_muted) {
                buzzer_muted = true;
                buzzer_set(false);
                // Clear latched fall and send "false alarm"
                if (last_alert_fall || last_alert_pulse || last_alert_temp) {
                    send_event_false_alarm();
                }
                fall_latched = false;
                ESP_LOGI(TAG, "Local button: buzzer muted & fall cleared");
                if (buzzer_power_param) {
                    esp_rmaker_param_val_t v = esp_rmaker_bool(true);
                    esp_rmaker_param_update_and_report(buzzer_power_param, v);
                }
            }
        }

        // -------- FALL DETECTION (Y-axis + magnitude, LATCHED) --------
        if (fall_enabled) {
            if (mpu6050_read_accel(&ax, &ay, &az) == ESP_OK) {
                float mag    = sqrtf(ax * ax + ay * ay + az * az);  // total |a|
                float ay_abs = fabsf(ay);                            // Y-axis abs

                ESP_LOGD(TAG, "Accel: ax=%.2f ay=%.2f az=%.2f |a|=%.2f",
                         ax, ay, az, mag);

                if ((mag > FALL_G_THRESHOLD) || (ay_abs > FALL_Y_THRESHOLD)) {
                    fall_latched = true;  // latch once triggered
                    ESP_LOGW(TAG, "Fall detected! ay=%.2f |a|=%.2f", ay, mag);
                }
            }
        }

        if (fall_latched) {
            alert_fall = true;
        }

        // -------- PULSE MONITOR (BPM) --------
        if (pulse_enabled) {
            bpm = read_pulse_bpm_simplified();
            if (bpm <= 0.1f) {
                ESP_LOGI(TAG, "No valid pulse signal");
            } else {
                ESP_LOGI(TAG, "Pulse approx BPM: %.1f", bpm);
                if (bpm > HIGH_BPM_THRESHOLD) {
                    alert_pulse = true;
                    ESP_LOGW(TAG, "High BPM detected: %.1f", bpm);
                }

                // Update RainMaker live BPM param
                if (pulse_value_param) {
                    esp_rmaker_param_val_t v = esp_rmaker_float(bpm);
                    esp_rmaker_param_update_and_report(pulse_value_param, v);
                }
            }
        }

        // -------- TEMPERATURE MONITOR (°C) --------
        if (temp_enabled) {
            temp_c = read_temperature_c();
            ESP_LOGD(TAG, "Temp: %.2f C", temp_c);
            if (temp_c > HIGH_TEMP_THRESHOLD_C) {
                alert_temp = true;
                ESP_LOGW(TAG, "High Temp detected: %.2f C", temp_c);
            }

            // Update RainMaker live temp param
            if (temp_value_param) {
                esp_rmaker_param_val_t v = esp_rmaker_float(temp_c);
                esp_rmaker_param_update_and_report(temp_value_param, v);
            }
        }

        any_alert = alert_fall || alert_pulse || alert_temp;

        // -------- Edge-based notifications --------
        if (alert_fall && !last_alert_fall) {
            send_event_fall_detected();
        }
        if (alert_pulse && !last_alert_pulse && bpm > 0.1f) {
            send_event_high_bpm(bpm);
        }
        if (alert_temp && !last_alert_temp) {
            send_event_high_temp(temp_c);
        }

        last_alert_fall  = alert_fall;
        last_alert_pulse = alert_pulse;
        last_alert_temp  = alert_temp;

        // -------- Decide OLED text --------
        if (any_alert) {
            // Priority: Fall > Temp > Pulse
            if (alert_fall) {
                strncpy(line1, "FALL DETECTED", sizeof(line1) - 1);
                strncpy(line2, "Calling for help", sizeof(line2) - 1);
            } else if (alert_temp) {
                strncpy(line1, "High Temp!", sizeof(line1) - 1);
                strncpy(line2, "Check patient", sizeof(line2) - 1);
            } else if (alert_pulse) {
                strncpy(line1, "HIGH BPM!", sizeof(line1) - 1);
                strncpy(line2, "Check patient", sizeof(line2) - 1);
            }
        } else {
            // No alerts -> show live readings depending on what is enabled
            if (pulse_enabled && temp_enabled) {
                snprintf(line1, sizeof(line1), "Pulse: %.0f BPM", bpm);
                snprintf(line2, sizeof(line2), "Temp: %.1f C", temp_c);
            } else if (pulse_enabled) {
                snprintf(line1, sizeof(line1), "Pulse: %.0f BPM", bpm);
                strncpy(line2, "Pulse Monitor ON", sizeof(line2) - 1);
            } else if (temp_enabled) {
                snprintf(line1, sizeof(line1), "Temp: %.1f C", temp_c);
                strncpy(line2, "Temp Monitor ON", sizeof(line2) - 1);
            } else {
                strncpy(line1, "Health Monitor", sizeof(line1) - 1);
                strncpy(line2, "Idle", sizeof(line2) - 1);
            }
        }

        // -------- Buzzer control --------
        if (any_alert && !buzzer_muted) {
            buzzer_set(true);
        } else {
            buzzer_set(false);
        }

        // -------- Update OLED --------
        oled_draw_status(line1, line2);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_DELAY_MS));
    }
}

/* ---------- RainMaker init (node + devices) ---------- */

static void rainmaker_init_and_start(void)
{
    // Initialize Wi-Fi helper (from esp-rainmaker examples)
    app_wifi_init();

    // Initialize RainMaker node
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(
        &rainmaker_cfg,
        "Health Monitor Node",
        "HealthMonitor");
    if (!node) {
        ESP_LOGE(TAG, "Could not init RainMaker node");
        vTaskDelay(pdMS_TO_TICKS(5000));
        abort();
    }

    // FALL DETECTOR device
    fall_device = esp_rmaker_device_create("Fall Detector",
                                           ESP_RMAKER_DEVICE_SWITCH,
                                           NULL);
    esp_rmaker_device_add_cb(fall_device, switch_write_cb, NULL);
    esp_rmaker_device_add_param(fall_device,
        esp_rmaker_name_param_create("name", "Fall Detector"));
    esp_rmaker_param_t *fall_power_param =
        esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(fall_device, fall_power_param);
    esp_rmaker_device_assign_primary_param(fall_device, fall_power_param);
    esp_rmaker_node_add_device(node, fall_device);

    // PULSE MONITOR device
    pulse_device = esp_rmaker_device_create("Pulse Monitor",
                                            ESP_RMAKER_DEVICE_SWITCH,
                                            NULL);
    esp_rmaker_device_add_cb(pulse_device, switch_write_cb, NULL);
    esp_rmaker_device_add_param(pulse_device,
        esp_rmaker_name_param_create("name", "Pulse Monitor"));

    // ON/OFF switch
    esp_rmaker_param_t *pulse_power_param =
        esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(pulse_device, pulse_power_param);
    esp_rmaker_device_assign_primary_param(pulse_device, pulse_power_param);

    // Live BPM value param (type=NULL => custom float)
    pulse_value_param = esp_rmaker_param_create(
        "BPM",
        NULL,
        esp_rmaker_float(0.0f),
        PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    esp_rmaker_device_add_param(pulse_device, pulse_value_param);

    esp_rmaker_node_add_device(node, pulse_device);

    // TEMP MONITOR device
    temp_device = esp_rmaker_device_create("Temp Monitor",
                                           ESP_RMAKER_DEVICE_SWITCH,
                                           NULL);
    esp_rmaker_device_add_cb(temp_device, switch_write_cb, NULL);
    esp_rmaker_device_add_param(temp_device,
        esp_rmaker_name_param_create("name", "Temp Monitor"));
    esp_rmaker_param_t *temp_power_param =
        esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(temp_device, temp_power_param);
    esp_rmaker_device_assign_primary_param(temp_device, temp_power_param);

    // Live Temperature value param
    temp_value_param = esp_rmaker_param_create(
        "Temperature",
        NULL,
        esp_rmaker_float(0.0f),
        PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    esp_rmaker_device_add_param(temp_device, temp_value_param);

    esp_rmaker_node_add_device(node, temp_device);

    // BUZZER MUTE device (Help Found)
    buzzer_device = esp_rmaker_device_create("Buzzer Mute",
                                             ESP_RMAKER_DEVICE_SWITCH,
                                             NULL);
    esp_rmaker_device_add_cb(buzzer_device, switch_write_cb, NULL);
    esp_rmaker_device_add_param(buzzer_device,
        esp_rmaker_name_param_create("name", "Buzzer Mute"));
    buzzer_power_param =
        esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, false);
    esp_rmaker_device_add_param(buzzer_device, buzzer_power_param);
    esp_rmaker_device_assign_primary_param(buzzer_device, buzzer_power_param);
    esp_rmaker_node_add_device(node, buzzer_device);

    // Start RainMaker agent
    ESP_ERROR_CHECK(esp_rmaker_start());

    // Start Wi-Fi provisioning/connection. POP_TYPE_RANDOM is the usual choice.
    app_wifi_start(POP_TYPE_RANDOM);

    ESP_LOGI(TAG, "RainMaker started. Use phone app to provision & control.");
}

/* ---------- app_main ---------- */

void app_main(void)
{
    // Init low-level drivers (I2C, ADC, OLED, buzzer, button)
    app_driver_init();

    // NVS (needed for Wi-Fi, RainMaker)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    // Start RainMaker (Wi-Fi + cloud + voice assistants)
    rainmaker_init_and_start();

    // Start main sensor / logic task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}

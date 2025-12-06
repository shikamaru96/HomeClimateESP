/* Smart Home Climate Monitoring & Alert System
 *
 * - ESP-IDF + FreeRTOS + ESP RainMaker
 * - Temperature + Humidity monitoring
 * - Local alerts using LEDs + Buzzer
 * - Cloud parameters via RainMaker
 * - OTA enabled
 * - Ready for Google Assistant / Alexa integration via RainMaker
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_common_events.h>

#include <app_network.h>
#include <app_insights.h>

#include "driver/gpio.h"
#include "app_priv.h"     // from RainMaker examples/common 
#include "esp_rom_sys.h"   // for esp_rom_delay_us()
#include "esp_app_desc.h"

static const char *TAG = "app_main";

/* -------------------- GPIO CONFIG -------------------- */
#define GPIO_LED_OK      GPIO_NUM_2   // Green LED for normal status
#define GPIO_LED_TEMP   GPIO_NUM_3   // Red LED for temperature alert
#define GPIO_BUZZER      GPIO_NUM_4   // Buzzer output
#define DHT11_GPIO   GPIO_NUM_5   // Temperature and humidity sensor
#define GPIO_LED_HUM    GPIO_NUM_6   // Blue LED for humidity alert

#define FIRMWARE_VERSION "2.0.0"

/* -------------------- GLOBAL STATE -------------------- */
typedef enum {
    MODE_NORMAL   = 0,
    MODE_HOT_DAY  = 1,
    MODE_COLD_DAY = 2,
} climate_mode_t;

static climate_mode_t g_mode = MODE_NORMAL;

// Default thresholds for each mode
#define NORMAL_TEMP_THRESHOLD   30.0f
#define NORMAL_HUM_THRESHOLD    80.0f
#define HOTDAY_TEMP_THRESHOLD   33.0f
#define HOTDAY_HUM_THRESHOLD    86.0f
#define COLDDAY_TEMP_THRESHOLD   25.0f   
#define COLDDAY_HUM_THRESHOLD    40.0f   

// existing globals
static float g_temperature = 0.0f;
static float g_humidity    = 0.0f;
static float g_temp_threshold  = NORMAL_TEMP_THRESHOLD;
static float g_hum_threshold   = NORMAL_HUM_THRESHOLD;

// add handle for Mode param
static esp_rmaker_param_t *mode_param = NULL;

// Alarm configuration
static bool g_alarm_enabled  = true;   // master alarm flag
static bool g_buzzer_enabled = true;   // allow buzzer sound

// Alert status string
static char g_alert_status[32] = "OK";

// Small flag to request a test beep from write_cb()
static volatile bool g_test_alarm_request = false;

static float last_temp_reported = -1000.0f;
static float last_hum_reported  = -1000.0f;

static char last_alert_status[32] = "";

// RainMaker handles so tasks can update parameters
static esp_rmaker_device_t *climate_device;
static esp_rmaker_param_t  *temp_param;
static esp_rmaker_param_t  *hum_param;
static esp_rmaker_param_t  *temp_threshold_param;
static esp_rmaker_param_t  *hum_threshold_param;
static esp_rmaker_param_t  *alarm_enabled_param;
static esp_rmaker_param_t  *buzzer_enabled_param;
static esp_rmaker_param_t  *alert_status_param;

/* -------------------- HARDWARE-SPECIFIC STUBS -------------------- */
// ==================== DHT11 DRIVER (BIT-BANGED) ====================

static void sensor_hw_init(void)
{
    // DHT11 idle state: data line input with pull-up (external or internal)
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DHT11_GPIO),
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set high initially
    gpio_set_level(DHT11_GPIO, 1);
}

// Helper: switch pin to output/input quickly
static inline void dht11_set_output(void)
{
    gpio_set_direction(DHT11_GPIO, GPIO_MODE_OUTPUT);
}

static inline void dht11_set_input(void)
{
    gpio_set_direction(DHT11_GPIO, GPIO_MODE_INPUT);
}

// Read one bit (blocking ~70µs)
static int dht11_read_bit(void)
{
    int retry = 0;

    // Each bit starts with ~50µs LOW
    while (!gpio_get_level(DHT11_GPIO)) {
        esp_rom_delay_us(1);
        if (++retry > 100) return -1; // timeout
    }

    // Then it goes HIGH: length of HIGH = bit value
    retry = 0;
    esp_rom_delay_us(30);  // wait 30µs, then sample

    int bit = gpio_get_level(DHT11_GPIO);

    // Wait for line to go low again (end of this bit)
    while (gpio_get_level(DHT11_GPIO)) {
        esp_rom_delay_us(1);
        if (++retry > 100) break; // just safety
    }

    return bit;
}

static bool dht11_read_raw(uint8_t data[5])
{
    int i, j;

    // Send start signal
    dht11_set_output();
    gpio_set_level(DHT11_GPIO, 0);
    esp_rom_delay_us(20000); // >18ms low
    gpio_set_level(DHT11_GPIO, 1);
    esp_rom_delay_us(30);    // 20–40µs high
    dht11_set_input();

    // Wait for sensor response: LOW 80µs, HIGH 80µs
    int retry = 0;
    while (gpio_get_level(DHT11_GPIO)) {  // wait for LOW
        esp_rom_delay_us(1);
        if (++retry > 100) return false;
    }
    retry = 0;
    while (!gpio_get_level(DHT11_GPIO)) { // wait for HIGH
        esp_rom_delay_us(1);
        if (++retry > 100) return false;
    }
    retry = 0;
    while (gpio_get_level(DHT11_GPIO)) {  // wait for LOW (start of bits)
        esp_rom_delay_us(1);
        if (++retry > 100) return false;
    }

    // Now read 40 bits → 5 bytes
    for (i = 0; i < 5; i++) {
        uint8_t byte = 0;
        for (j = 0; j < 8; j++) {
            int bit = dht11_read_bit();
            if (bit < 0) return false;
            byte <<= 1;
            if (bit) byte |= 0x01;
        }
        data[i] = byte;
    }

    // Switch back to output high (idle)
    dht11_set_output();
    gpio_set_level(DHT11_GPIO, 1);

    // Checksum
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) {
        return false;
    }
    return true;
}

static bool sensor_hw_read(float *out_temp, float *out_hum)
{
    uint8_t data[5] = {0};

    if (!dht11_read_raw(data)) {
        ESP_LOGW(TAG, "DHT11 read failed (checksum or timeout).");
        return false;
    }

    // For DHT11: data[0] = RH int, data[1] = RH decimal (usually 0)
    //            data[2] = Temp int, data[3] = Temp decimal (usually 0)
    uint8_t rh_int    = data[0];
    uint8_t temp_int  = data[2];

    *out_hum  = (float)rh_int;
    *out_temp = (float)temp_int;

//    ESP_LOGI(TAG, "DHT11: Temp=%.1f°C, Hum=%.1f%%", *out_temp, *out_hum);
    return true;
}

// Example keypad init (4x4 or similar matrix keypad)
static void keypad_hw_init(void)
{
    // TODO: configure keypad GPIOs here
}

// Example keypad handling (polling, decode key → change thresholds, etc.)
static void keypad_handle_once(void)
{
    // TODO: scan keypad and update g_temp_threshold / g_hum_threshold, etc.
    // For now, no-op.
}

/* -------------------- LED & BUZZER HELPERS -------------------- */

static void io_alert_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_LED_OK)   |
                        (1ULL << GPIO_LED_TEMP) |
                        (1ULL << GPIO_LED_HUM)  |
                        (1ULL << GPIO_BUZZER),
        .pull_down_en = 0,
        .pull_up_en   = 0,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Start in "normal" state
    gpio_set_level(GPIO_LED_OK,   1);  // green ON
    gpio_set_level(GPIO_LED_TEMP, 0);  // red OFF
    gpio_set_level(GPIO_LED_HUM,  0);  // blue OFF
    gpio_set_level(GPIO_BUZZER,   0);
}

static void set_normal_state(void)
{
    gpio_set_level(GPIO_LED_OK,    1);
//    gpio_set_level(GPIO_LED_ALERT, 0);
    gpio_set_level(GPIO_LED_TEMP,  0);
    gpio_set_level(GPIO_LED_HUM,  0);
    gpio_set_level(GPIO_BUZZER,    0);
}

static void set_alert_state(bool beep)
{
    gpio_set_level(GPIO_LED_OK,    0);
//    gpio_set_level(GPIO_LED_ALERT, 1);
    gpio_set_level(GPIO_LED_TEMP,  1);
    gpio_set_level(GPIO_LED_HUM,   1);
    if (g_alarm_enabled && g_buzzer_enabled && beep) {
        gpio_set_level(GPIO_BUZZER, 1);
    } else {
        gpio_set_level(GPIO_BUZZER, 0);
    }
}

// Simple non-blocking buzzer tick: called periodically
static void buzzer_pulse_once(void)
{
    static int counter = 0;
    if (g_test_alarm_request) {
        // One short beep (e.g. 200 ms) using periodic task calls
        if (counter == 0) {
            gpio_set_level(GPIO_BUZZER, 1);
        }
        counter++;
        if (counter > 5) {  // adjust based on task period
            gpio_set_level(GPIO_BUZZER, 0);
            g_test_alarm_request = false;
            counter = 0;
        }
    }
}

/* -------------------- ALERT LOGIC -------------------- */

static void update_alert_status_and_io(void)
{
    const char *new_status = "OK";
    bool high_temp = (g_temperature > g_temp_threshold);
    bool high_hum  = (g_humidity    > g_hum_threshold);

    if (high_temp && high_hum) {
        new_status = "HighTemp+Humidity";
    } else if (high_temp) {
        new_status = "HighTemp";
    } else if (high_hum) {
        new_status = "HighHumidity";
    } else {
        new_status = "OK";
    }

    // Copy to global buffer
    strncpy(g_alert_status, new_status, sizeof(g_alert_status) - 1);
    g_alert_status[sizeof(g_alert_status) - 1] = '\0';

    if (!g_alarm_enabled) {
        // Alarm disarmed: show OK, turn everything off
        strcpy(g_alert_status, "Disabled");
        gpio_set_level(GPIO_LED_OK,   0);
        gpio_set_level(GPIO_LED_TEMP, 0);
        gpio_set_level(GPIO_LED_HUM,  0);
        gpio_set_level(GPIO_BUZZER,   0);

        if (alert_status_param) {
            esp_rmaker_param_update_and_report(alert_status_param,
                esp_rmaker_str(g_alert_status));
        }
        return;
    }

    // --- LED logic ---

    if (strcmp(g_alert_status, "OK") == 0) {
        // Normal state: GREEN on, others off
        gpio_set_level(GPIO_LED_OK,   1);
        gpio_set_level(GPIO_LED_TEMP, 0);
        gpio_set_level(GPIO_LED_HUM,  0);
        gpio_set_level(GPIO_BUZZER,   0);
    } else {
        // Alert state: GREEN off
        gpio_set_level(GPIO_LED_OK, 0);

        // Temp alert → RED LED
        gpio_set_level(GPIO_LED_TEMP, high_temp ? 1 : 0);

        // Humidity alert → BLUE LED
        gpio_set_level(GPIO_LED_HUM,  high_hum  ? 1 : 0);

        // Buzzer if any alert and enabled
        if ((high_temp || high_hum) && g_buzzer_enabled) {
            gpio_set_level(GPIO_BUZZER, 1);
        } else {
            gpio_set_level(GPIO_BUZZER, 0);
        }
    }

    // Update RainMaker param
    if (alert_status_param) {
        esp_rmaker_param_val_t v = esp_rmaker_str(g_alert_status);
        esp_rmaker_param_update_and_report(alert_status_param, v);
    }

        if (strcmp(g_alert_status, last_alert_status) != 0) {
        strcpy(last_alert_status, g_alert_status);

        if (alert_status_param) {
            esp_rmaker_param_update_and_report(
                alert_status_param, esp_rmaker_str(g_alert_status));
        }
    }
}

static void apply_mode(climate_mode_t mode)
{
    g_mode = mode;

    if (mode == MODE_HOT_DAY) {
        g_temp_threshold = HOTDAY_TEMP_THRESHOLD;
        g_hum_threshold  = HOTDAY_HUM_THRESHOLD;
    } else if (mode == MODE_COLD_DAY) {
        g_temp_threshold = COLDDAY_TEMP_THRESHOLD;
        g_hum_threshold  = COLDDAY_HUM_THRESHOLD;
    } else {
        g_temp_threshold = NORMAL_TEMP_THRESHOLD;
        g_hum_threshold  = NORMAL_HUM_THRESHOLD;
    }

    // Push new thresholds to RainMaker so app shows updated values
    if (temp_threshold_param) {
        esp_rmaker_param_val_t v = esp_rmaker_float(g_temp_threshold);
        esp_rmaker_param_update_and_report(temp_threshold_param, v);
    }
    if (hum_threshold_param) {
        esp_rmaker_param_val_t v = esp_rmaker_float(g_hum_threshold);
        esp_rmaker_param_update_and_report(hum_threshold_param, v);
    }
}


/* -------------------- RAINMAKER WRITE CALLBACK -------------------- */

static esp_err_t write_cb(const esp_rmaker_device_t *device,
                          const esp_rmaker_param_t *param,
                          const esp_rmaker_param_val_t val,
                          void *priv_data,
                          esp_rmaker_write_ctx_t *ctx)
{
    const char *param_name = esp_rmaker_param_get_name(param);
    ESP_LOGI(TAG, "Write request: device=%s, param=%s",
             esp_rmaker_device_get_name(device), param_name);

    if (strcmp(param_name, "TempThreshold") == 0) {
        g_temp_threshold = val.val.f;
        ESP_LOGI(TAG, "New TempThreshold=%.2f", g_temp_threshold);
        esp_rmaker_param_update_and_report(param, val);
        return ESP_OK;
    }

    if (strcmp(param_name, "HumThreshold") == 0) {
        g_hum_threshold = val.val.f;
        ESP_LOGI(TAG, "New HumThreshold=%.2f", g_hum_threshold);
        esp_rmaker_param_update_and_report(param, val);
        return ESP_OK;
    }

    if (strcmp(param_name, "AlarmEnabled") == 0) {
        g_alarm_enabled = val.val.b;
        ESP_LOGI(TAG, "AlarmEnabled=%d", g_alarm_enabled);
        esp_rmaker_param_update_and_report(param, val);
        return ESP_OK;
    }

    if (strcmp(param_name, "BuzzerEnabled") == 0) {
        g_buzzer_enabled = val.val.b;
        ESP_LOGI(TAG, "BuzzerEnabled=%d", g_buzzer_enabled);
        esp_rmaker_param_update_and_report(param, val);
        return ESP_OK;
    }

    if (strcmp(param_name, "TestAlarm") == 0) {
        if (val.val.b) {
            ESP_LOGI(TAG, "TestAlarm requested.");
            g_test_alarm_request = true;   // handled in cloud_task
        }
        // Auto-reset param to false
        esp_rmaker_param_val_t off_v = esp_rmaker_bool(false);
        esp_rmaker_param_update_and_report(param, off_v);
        return ESP_OK;
    }

    if (strcmp(param_name, "Mode") == 0) {
        const char *mode_str = val.val.s ? val.val.s : "Normal";
        ESP_LOGI(TAG, "Mode change requested: %s", mode_str);

        if (strcmp(mode_str, "HotDay") == 0) {
            apply_mode(MODE_HOT_DAY);
        } else if (strcmp(mode_str, "ColdDay") == 0) {
            apply_mode(MODE_COLD_DAY);
        } else {
            apply_mode(MODE_NORMAL);
        }

        // Confirm mode value back to cloud (normalized)
        const char *normalized = "Normal";
        if (g_mode == MODE_HOT_DAY)  normalized = "HotDay";
        if (g_mode == MODE_COLD_DAY) normalized = "ColdDay";

        esp_rmaker_param_val_t v = esp_rmaker_str(normalized);
        esp_rmaker_param_update_and_report(param, v);

        return ESP_OK;
    }


    return ESP_OK;
}

/* -------------------- EVENT HANDLER (RainMaker & OTA) -------------------- */

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
//                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            default:
 //               ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            default:
 //               ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_NETWORK_EVENT) {
        switch (event_id) {
            case APP_NETWORK_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            default:
 //               ESP_LOGW(TAG, "Unhandled App Network Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch(event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "OTA starting.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA failed.");
                break;
            default:
 //               ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

/* -------------------- FREERTOS TASKS -------------------- */

static void sensor_task(void *arg)
{
    while (1) {
        float t, h;
        if (sensor_hw_read(&t, &h)) {
            g_temperature = t;
            g_humidity    = h;
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2s
    }
}

/*static void keypad_task(void *arg)
{
    while (1) {
        keypad_handle_once();
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms
    }
}*/

static void cloud_task(void *arg)
{
    while (1) {
        // Temperature
        if (temp_param) {
            if (fabsf(g_temperature - last_temp_reported) > 0.1f) { // 0.1°C hysteresis
                last_temp_reported = g_temperature;
                esp_rmaker_param_update_and_report(
                    temp_param, esp_rmaker_float(g_temperature));
            }
        }

        // Humidity
        if (hum_param) {
            if (fabsf(g_humidity - last_hum_reported) > 1.0f) { // 1% RH hysteresis
                last_hum_reported = g_humidity;
                esp_rmaker_param_update_and_report(
                    hum_param, esp_rmaker_float(g_humidity));
            }
        }

        // Thresholds: only push if changed (e.g. keypad or Alexa)
        static float last_temp_th = -1;
        static float last_hum_th  = -1;

        if (temp_threshold_param && g_temp_threshold != last_temp_th) {
            last_temp_th = g_temp_threshold;
            esp_rmaker_param_update_and_report(
                temp_threshold_param, esp_rmaker_float(g_temp_threshold));
        }

        if (hum_threshold_param && g_hum_threshold != last_hum_th) {
            last_hum_th = g_hum_threshold;
            esp_rmaker_param_update_and_report(
                hum_threshold_param, esp_rmaker_float(g_hum_threshold));
        }

        buzzer_pulse_once();
        update_alert_status_and_io();

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
/* -------------------- MAIN: app_main -------------------- */

void app_main(void)
{
    esp_err_t err;

//    esp_log_level_set("esp_rmaker_param", ESP_LOG_NONE);

    esp_rmaker_console_init();

    // Init hardware drivers (if you have other drivers, call app_driver_init() etc.)
    sensor_hw_init();
    keypad_hw_init();
    io_alert_init();
    set_normal_state();

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize Network (Wi-Fi + provisioning)
    app_network_init();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT,        ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT,   ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT,    ESP_EVENT_ANY_ID, &event_handler, NULL));

    // Initialise RainMaker node
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,   // Set true if using SNTP/time-based scheduling
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg,
                                                   "SmartHomeClimate",
                                                   "home_climate_controller");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    // Set firmware version properly here:
    esp_rmaker_node_add_fw_version(node, FIRMWARE_VERSION);
    // Just log the macro – no getter needed
    ESP_LOGI(TAG, "FW version at boot (macro): %s", FIRMWARE_VERSION);

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "AppDesc version from binary: %s", app_desc->version);

    // Create a single device to represent the climate system
    climate_device = esp_rmaker_device_create("HomeClimate",
                                              "home_climate",
                                              NULL);

    apply_mode(MODE_NORMAL);

    // Attach write callback
    esp_rmaker_device_add_cb(climate_device, write_cb, NULL);

    // Name parameter
    esp_rmaker_device_add_param(climate_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "HomeClimate"));

    // Temperature (read-only)
    temp_param = esp_rmaker_param_create("Temperature", "esp.param.temperature",
                                         esp_rmaker_float(0.0),
                                         PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(temp_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(climate_device, temp_param);

    // Humidity (read-only)
    hum_param = esp_rmaker_param_create("Humidity", "esp.param.humidity",
                                        esp_rmaker_float(0.0),
                                        PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(hum_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(climate_device, hum_param);

    // Temperature threshold (R/W)
    temp_threshold_param = esp_rmaker_param_create("TempThreshold", "esp.param.temp_threshold",
                                                   esp_rmaker_float(g_temp_threshold),
                                                   PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(temp_threshold_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_device_add_param(climate_device, temp_threshold_param);

    // Humidity threshold (R/W)
    hum_threshold_param = esp_rmaker_param_create("HumThreshold", "esp.param.hum_threshold",
                                                  esp_rmaker_float(g_hum_threshold),
                                                  PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(hum_threshold_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_device_add_param(climate_device, hum_threshold_param);

    // AlarmEnabled (R/W)
    alarm_enabled_param = esp_rmaker_param_create("AlarmEnabled", "esp.param.alarm_enabled",
                                                  esp_rmaker_bool(g_alarm_enabled),
                                                  PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(alarm_enabled_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(climate_device, alarm_enabled_param);

    // BuzzerEnabled (R/W)
    buzzer_enabled_param = esp_rmaker_param_create("BuzzerEnabled", "esp.param.buzzer_enabled",
                                                   esp_rmaker_bool(g_buzzer_enabled),
                                                   PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(buzzer_enabled_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(climate_device, buzzer_enabled_param);

    // AlertStatus (read-only, text box)
    alert_status_param = esp_rmaker_param_create("AlertStatus", "esp.param.alert_status",
                                                 esp_rmaker_str("OK"),
                                                 PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(alert_status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(climate_device, alert_status_param);

    // Primary param (for voice/home screen): choose AlarmEnabled
    esp_rmaker_device_assign_primary_param(climate_device, alarm_enabled_param);

        // ---- Mode param: Normal / HotDay ----
    static const char *mode_str_list[] = { "Normal", "HotDay", "ColdDay" };

    mode_param = esp_rmaker_param_create(
        "Mode",                 // name shown in app
        "esp.param.mode",       // custom type
        esp_rmaker_str("Normal"),
        PROP_FLAG_READ | PROP_FLAG_WRITE
    );
    esp_rmaker_param_add_ui_type(mode_param, ESP_RMAKER_UI_DROPDOWN);
    esp_rmaker_param_add_valid_str_list(
        mode_param,
        mode_str_list,
        sizeof(mode_str_list) / sizeof(mode_str_list[0])
    );
    esp_rmaker_device_add_param(climate_device, mode_param);


    // Add device to node
    esp_rmaker_node_add_device(node, climate_device);

    // Enable OTA
    esp_rmaker_ota_enable_default();

    // Optional services
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();

    // Start RainMaker agent
    esp_rmaker_start();

    // Start network (connect or start provisioning)
    err = app_network_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    (void)err; // ignore if not using manufacturing data
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    ESP_LOGI(TAG, "Smart Home Climate Monitoring system started.");

    // Create tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
//    xTaskCreate(keypad_task, "keypad_task", 4096, NULL, 5, NULL);
    xTaskCreate(cloud_task,  "cloud_task",  4096, NULL, 5, NULL);
}

/* Temperature Control
- Read ADC (they are trash on ESP32 so we average a lot)
- Convert 0-1V Temperature/Humidity signal to data and send to MQTT Broker
- Control a Relay or Tasmota Smart Plug which has a Heater connected
- Read Temperature Setpoint from potentiometer
- Set Hysteresis via NVS command
*/

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_console.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_chip_info.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "temp_ctrl.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "nvs.h"

#define ADC_VOLT_MAX 1087 // Voltage (mV) when the ADC shows ADC_MAX
#define ADC_0V 12         // Value of the ADC without any input
#define ADC_MAX 2047

#define ADC_CHANNEL_POT ADC_CHANNEL_3 // GPIO39
#define ADC_CHANNEL_T1 ADC_CHANNEL_4  // GPIO32
#define ADC_CHANNEL_RH1 ADC_CHANNEL_5 // GPIO33
#define ADC_CHANNEL_T2 ADC_CHANNEL_6  // GPIO34
#define ADC_CHANNEL_RH2 ADC_CHANNEL_7 // GPIO35

#define ADC_CHANNEL_POT_OFFSET 0  // MILLIVOLT
#define ADC_CHANNEL_T1_OFFSET 28  // MILLIVOLT
#define ADC_CHANNEL_RH1_OFFSET 36 // MILLIVOLT
#define ADC_CHANNEL_T2_OFFSET 28  // MILLIVOLT
#define ADC_CHANNEL_RH2_OFFSET 40 // MILLIVOLT

#define SETPOINT_MIN 18
#define SETPOINT_MAX 22

#define SENSOR_DETECT_MIN_T_MV 250  // -10°C
#define SENSOR_DETECT_MIN_RH_MV 100 //  10%

#define RELAY_PIN GPIO_NUM_18
#define HEATING_CONTROL_ENABLE_PIN GPIO_NUM_19

const static char *TAG = "Temperature Control";

static uint8_t get_u8_from_nvs(const char *key);
static float map(float x, float in_min, float in_max, float out_min, float out_max);
static uint16_t adc_to_mV(uint16_t adc_val);
static uint16_t read_adc(adc_channel_t channel, uint8_t mean, uint16_t mean_read_delay);
static void heater_control(float temperature);
static void check_enabled_sensors(void);
static uint8_t heater_control_enabled(void);
static void set_relay_state(uint8_t state);
static uint8_t get_relay_state(void);

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_channel_t channels[5] = {ADC_CHANNEL_POT, ADC_CHANNEL_T1, ADC_CHANNEL_RH1, ADC_CHANNEL_T2, ADC_CHANNEL_RH2};
static uint16_t channel_mV_offset[5] = {ADC_CHANNEL_POT_OFFSET, ADC_CHANNEL_T1_OFFSET, ADC_CHANNEL_RH1_OFFSET, ADC_CHANNEL_T2_OFFSET, ADC_CHANNEL_RH2_OFFSET};
static uint16_t channel_res_raw[5] = {0, 0, 0, 0, 0};
static uint16_t channel_res_mV[5] = {0, 0, 0, 0, 0};
static float t1, t2, rh1, rh2;
static bool sensor1_enabled = false;
static bool sensor2_enabled = false;
static uint16_t temperature_setpoint = 30;

static uint8_t temperature_hysteresis_min = 1; // nvs_set t_hyst_min u8 -v 1
static uint8_t temperature_hysteresis_max = 1; // nvs_set t_hyst_max u8 -v 1
static char current_namespace[16] = "storage";

void mqtt_app_start(void);
int mqtt_publish(float t1, float rh1, float t2, float rh2, float setpoint, uint8_t t_hyst_min, uint8_t t_hyst_max, uint8_t relay_state, uint8_t control_state);
esp_err_t set_tasmota_relay(uint8_t onoff);
SSD1306_t dev;
char lines[8][20];

void initialize_temperature_control(void)
{
    mqtt_app_start();

    gpio_set_direction(RELAY_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(HEATING_CONTROL_ENABLE_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HEATING_CONTROL_ENABLE_PIN, GPIO_PULLUP_ONLY);

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_11,
        .atten = ADC_ATTEN_DB_0,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));

    temperature_hysteresis_min = get_u8_from_nvs("t_hyst_min");
    temperature_hysteresis_max = get_u8_from_nvs("t_hyst_max");
    ESP_LOGI(TAG, "t_hyst_min=%d, t_hyst_max=%d", temperature_hysteresis_min, temperature_hysteresis_max);

    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "Searching Sensors...", 21, true);

    check_enabled_sensors();

    if (sensor1_enabled)
        ssd1306_display_text(&dev, 1, "Sensor 1 OK", 12, false);
    if (sensor2_enabled)
        ssd1306_display_text(&dev, 2, "Sensor 2 OK", 12, false);
    if (!sensor1_enabled && !sensor2_enabled)
        ssd1306_display_text(&dev, 1, "NO SENSOR FOUND!", 17, false);

    vTaskDelay(pdMS_TO_TICKS(1500));
    ssd1306_clear_screen(&dev, false);
}

void temperature_control_loop(void)
{
    static uint8_t _cnt_sensor_loop = 0;
    static uint8_t _cnt_avg_measurements = 0;
    static uint8_t avg_measurements = 50;

    static uint8_t _cnt_poti_loop = 0;
    static uint8_t _mode_changed = 0;
    static uint8_t _curr = 0;
    static uint8_t _prev = 1;

    if (_cnt_poti_loop++ >= 2)
    {
        _cnt_poti_loop = 0;

        uint16_t val = read_adc(channels[0], 30, 3);
        channel_res_mV[0] = adc_to_mV(val) + channel_mV_offset[0];

        // check if mode changed so we can update the display immediately
        _curr = heater_control_enabled();
        _mode_changed = 0;
        if (_prev != _curr)
        {
            _mode_changed = 1;
        }
        _prev = _curr;

        if (heater_control_enabled())
        {
            uint16_t _tmp = (uint16_t)map(channel_res_mV[0], 0, 1000, SETPOINT_MAX, SETPOINT_MIN);
            if (_tmp != temperature_setpoint || _mode_changed)
            {
                temperature_setpoint = _tmp;
                ESP_LOGI(TAG, "Changed temperature setpoint to: %d", temperature_setpoint);
                ssd1306_clear_line(&dev, 6, false);
                sprintf(&lines[6][0], "T SOLL: %dC", temperature_setpoint);
                ssd1306_display_text(&dev, 6, lines[6], strlen(lines[6]), true);
            }
        }
        else
        {
            // use poti to set relay on/off when heating control is disabled
            uint8_t _state = (uint8_t)map(channel_res_mV[0], 0, 1000, 1, 0);

            if (_state != get_relay_state() || _mode_changed)
            {
                ESP_LOGI(TAG, "Set Relay manually: %d", _state);
                ssd1306_clear_line(&dev, 6, false);
                char *_tmp = "OFF";
                if (_state)
                    _tmp = "ON";
                sprintf(&lines[6][0], "RELAIS: %s", _tmp);
                ssd1306_display_text(&dev, 6, lines[6], strlen(lines[6]), true);
                set_relay_state(_state);
            }
        }
    }

    if (_cnt_sensor_loop++ >= 4)
    {
        _cnt_sensor_loop = 0;

        // index 0 = poti channel, we only need sensor channels
        // ESP32 ADC is fuckin trash so average a LOT
        if (_cnt_avg_measurements++ <= avg_measurements)
        {
            for (size_t i = 1; i < 5; i++)
            {
                uint16_t val = read_adc(channels[i], 20, 2);
                channel_res_raw[i] = val;
                channel_res_mV[i] = adc_to_mV(val) + channel_mV_offset[i];
            }
            t1 += map(channel_res_mV[1], 0, 1000, -40, 80);
            rh1 += map(channel_res_mV[2], 0, 1000, 0, 100);
            t2 += map(channel_res_mV[3], 0, 1000, -40, 80);
            rh2 += map(channel_res_mV[4], 0, 1000, 0, 100);
        }
        else
        {
            _cnt_avg_measurements = 0;
            t1 = t1 / avg_measurements;
            rh1 = rh1 / avg_measurements;
            t2 = t2 / avg_measurements;
            rh2 = rh2 / avg_measurements;

            if (sensor1_enabled)
            {
                ssd1306_clear_line(&dev, 0, false);
                ssd1306_clear_line(&dev, 1, false);
                sprintf(&lines[0][0], "T1 : %.1fC", t1);
                ssd1306_display_text(&dev, 0, lines[0], strlen(lines[0]), false);
                sprintf(&lines[1][0], "RH1: %.0f%%", rh1);
                ssd1306_display_text(&dev, 1, lines[1], strlen(lines[1]), false);
            }

            if (sensor2_enabled)
            {
                ssd1306_clear_line(&dev, 3, false);
                ssd1306_clear_line(&dev, 4, false);
                sprintf(&lines[3][0], "T2 : %.1fC", t2);
                ssd1306_display_text(&dev, 3, lines[3], strlen(lines[3]), false);
                sprintf(&lines[4][0], "RH2: %.0f%%", rh2);
                ssd1306_display_text(&dev, 4, lines[4], strlen(lines[4]), false);
            }

            // ESP_LOGI(TAG,"RAW   : %04d\t%04d\t%04d\t%04d\t%04d", channel_res_raw[0], channel_res_raw[1], channel_res_raw[2], channel_res_raw[3], channel_res_raw[4]);
            // ESP_LOGI(TAG,"mV(O) : %04d\t%04d\t%04d\t%04d\t%04d", channel_res_mV[0], channel_res_mV[1], channel_res_mV[2], channel_res_mV[3], channel_res_mV[4]);
            // ESP_LOGI(TAG,"Value : %04d\t%2.1f\t%2.1f\t%2.1f\t%2.1f", 0, t1, rh1, t2, rh2);
            // ESP_LOGI(TAG,"===================================================================================================");

            // when both enabled, control with sensor 1
            if ((sensor1_enabled && !sensor2_enabled) || (sensor1_enabled && sensor2_enabled))
            {
                heater_control(t1);
            }
            else if (!sensor1_enabled && sensor2_enabled)
            {
                heater_control(t2);
            }
            else
            {
                ESP_LOGE(TAG, "Error in heater control. No Sensor connected?");
            }

            mqtt_publish(t1, rh1, t2, rh2, temperature_setpoint, temperature_hysteresis_min, temperature_hysteresis_max, get_relay_state(), heater_control_enabled());
            t1 = t2 = rh1 = rh2 = 0;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

static void heater_control(float temperature)
{
    if (!heater_control_enabled())
    {
        ESP_LOGW(TAG, "Heater Control Disabled");
        return;
    }

    if (temperature < temperature_setpoint - temperature_hysteresis_min)
    {
        if (!get_relay_state())
        {
            ESP_LOGI(TAG, "Temperature under setpoint, turning relay on");
        }
        set_relay_state(1);
    }
    else if (temperature > temperature_setpoint + temperature_hysteresis_max)
    {
        if (get_relay_state())
        {
            ESP_LOGI(TAG, "Temperature setpoint reached, turning relay off");
        }
        set_relay_state(0);
    }
}

static void check_enabled_sensors(void)
{
    // sensor 1 + 2
    for (size_t i = 1; i <= 4; i++)
    {
        uint16_t val = read_adc(channels[i], 40, 30);
        channel_res_raw[i] = val;
        channel_res_mV[i] = adc_to_mV(val) + channel_mV_offset[i];
    }

    ESP_LOGI(TAG, "Sensor1: %dmV, %dmV", channel_res_mV[1], channel_res_mV[2]);
    if (channel_res_mV[1] > SENSOR_DETECT_MIN_T_MV && channel_res_mV[2] > SENSOR_DETECT_MIN_RH_MV)
    {
        ESP_LOGI(TAG, "Sensor 1 enabled");
        sensor1_enabled = true;
    }

    ESP_LOGI(TAG, "Sensor2: %dmV, %dmV", channel_res_mV[3], channel_res_mV[4]);
    if (channel_res_mV[3] > SENSOR_DETECT_MIN_T_MV && channel_res_mV[4] > SENSOR_DETECT_MIN_RH_MV)
    {
        ESP_LOGI(TAG, "Sensor 2 enabled");
        sensor2_enabled = true;
    }
}

static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint16_t adc_to_mV(uint16_t raw_adc_value)
{
    if (raw_adc_value <= ADC_0V)
        return 0;
    return map(raw_adc_value, ADC_0V, ADC_MAX, 0, ADC_VOLT_MAX);
}

static uint16_t read_adc(adc_channel_t channel, uint8_t mean, uint16_t mean_read_delay)
{
    int tmp;
    uint16_t raw = 0;
    for (uint8_t n = 0; n < mean; n++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &tmp));
        raw += (uint16_t)tmp;
        vTaskDelay(pdMS_TO_TICKS(mean_read_delay));
    }
    return raw / mean;
}

static uint8_t heater_control_enabled(void)
{
    return (uint8_t)gpio_get_level(HEATING_CONTROL_ENABLE_PIN);
}

static void set_relay_state(uint8_t state)
{
    gpio_set_level(RELAY_PIN, state);
    // set_tasmota_relay(state);
}

static uint8_t get_relay_state(void)
{
    return (uint8_t)gpio_get_level(RELAY_PIN);
}

static uint8_t get_u8_from_nvs(const char *key)
{
    uint8_t value = 1;
    nvs_handle_t nvs;

    ESP_ERROR_CHECK(nvs_open(current_namespace, NVS_READONLY, &nvs));
    ESP_ERROR_CHECK(nvs_get_u8(nvs, key, &value));

    nvs_close(nvs);
    return value;
}

/* components/sensors/mq135_hal/mq135_hal.c */

/* TODO: Test this */
/* TODO: Move adc init to common/adc.h if needed */

#include "mq135_hal.h"
#include <math.h>
#include "webserver_tasks.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

/* Constants *******************************************************************/

const char    *mq135_tag                    = "MQ135";
const uint8_t  mq135_aout_pin               = GPIO_NUM_34;
const uint8_t  mq135_dout_pin               = GPIO_NUM_35;
const uint32_t mq135_polling_rate_ticks     = pdMS_TO_TICKS(1000);
const uint32_t mq135_warmup_time_ms         = 180000; /* 3-minute warm-up time */
const uint8_t  mq135_max_retries            = 4;
const uint32_t mq135_initial_retry_interval = pdMS_TO_TICKS(15000);
const uint32_t mq135_max_backoff_interval   = pdMS_TO_TICKS(480000);

/* Globals (Static) ***********************************************************/

/**
 * @brief ADC handle for the MQ135 sensor.
 *
 * This static global variable is used to manage the ADC configuration and
 * operations for reading the analog output (AOUT) of the MQ135 sensor. The ADC
 * handle is initialized during the `mq135_init` function and used in subsequen
 * read operations.
 */
static adc_oneshot_unit_handle_t adc1_handle;

/* Static (Private) Functions **************************************************/

/**
 * @brief Calculates gas concentration in ppm from raw ADC value.
 *
 * This function uses a mathematical formula to calculate the gas concentration
 * in parts per million (ppm) based on the raw ADC reading. The formula is an
 * approximation and requires calibration for accurate measurements.
 *
 * @param[in] raw_adc_value Raw ADC reading from the MQ135 sensor.
 *
 * @return Gas concentration in ppm.
 */
static float mq135_calculate_ppm(int raw_adc_value)
{
  float resistance = (4095.0 / raw_adc_value - 1.0) * 10.0; /* Assuming RL = 10k */
  float ppm        = 116.6020682 * pow(resistance / 10.0, -2.769034857); /* Example curve from datasheet */
  return ppm;
}

/* Public Functions ***********************************************************/

char *mq135_data_to_json(const mq135_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  cJSON_AddStringToObject(json, "sensor_type", "gas");
  cJSON_AddNumberToObject(json, "gas_concentration", data->gas_concentration);
  char *json_string = cJSON_PrintUnformatted(json);
  cJSON_Delete(json);
  return json_string;
}

esp_err_t mq135_init(void *sensor_data)
{
  mq135_data_t *mq135_data = (mq135_data_t *)sensor_data;
  ESP_LOGI(mq135_tag, "Initializing MQ135 Sensor");

  mq135_data->raw_adc_value      = 0;
  mq135_data->gas_concentration  = 0.0;
  mq135_data->state              = k_mq135_warming_up;
  mq135_data->warmup_start_ticks = xTaskGetTickCount();

  adc_oneshot_unit_init_cfg_t adc1_init_cfg = { .unit_id = ADC_UNIT_1 };
  esp_err_t ret = adc_oneshot_new_unit(&adc1_init_cfg, &adc1_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(mq135_tag, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
    return ret;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten    = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &chan_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(mq135_tag, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(mq135_tag, "MQ135 Initialization Complete");
  return ESP_OK;
}

esp_err_t mq135_read(mq135_data_t *sensor_data)
{
  mq135_data_t *mq135_data = (mq135_data_t *)sensor_data;

  TickType_t now_ticks = xTaskGetTickCount();
  if (now_ticks - mq135_data->warmup_start_ticks < pdMS_TO_TICKS(mq135_warmup_time_ms)) {
    mq135_data->state = k_mq135_warming_up;
    ESP_LOGW(mq135_tag, "Sensor is still warming up.");
    return ESP_FAIL;
  }

  int       raw_adc;
  esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_adc);
  if (ret != ESP_OK) {
    mq135_data->state = k_mq135_read_error;
    ESP_LOGE(mq135_tag, "Failed to read from ADC: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  mq135_data->raw_adc_value     = raw_adc;
  mq135_data->gas_concentration = mq135_calculate_ppm(raw_adc);

  ESP_LOGI(mq135_tag, "Raw ADC Value: %d, Gas Concentration: %.2f ppm", raw_adc, mq135_data->gas_concentration);

  mq135_data->state = k_mq135_ready;
  return ESP_OK;
}

void mq135_reset_on_error(mq135_data_t *sensor_data)
{
  if (sensor_data->state == k_mq135_read_error) {
    TickType_t current_ticks = xTaskGetTickCount();
    if ((current_ticks - sensor_data->warmup_start_ticks) > sensor_data->retry_interval) {
      ESP_LOGI(mq135_tag, "Attempting to reset MQ135 sensor");

      esp_err_t ret = mq135_init(sensor_data);
      if (ret == ESP_OK) {
        sensor_data->state          = k_mq135_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = mq135_initial_retry_interval;
        ESP_LOGI(mq135_tag, "MQ135 sensor reset successfully.");
      } else {
        sensor_data->retry_count++;
        if (sensor_data->retry_count >= mq135_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 > mq135_max_backoff_interval) ?
                                        mq135_max_backoff_interval : sensor_data->retry_interval * 2;
        }
      }

      sensor_data->warmup_start_ticks = current_ticks;
    }
  }
}

void mq135_tasks(void *sensor_data)
{
  mq135_data_t *mq135_data = (mq135_data_t *)sensor_data;
  while (1) {
    if (mq135_read(mq135_data) == ESP_OK) {
      char *json = mq135_data_to_json(mq135_data);
      send_sensor_data_to_webserver(json);
      free(json);
    } else {
      mq135_reset_on_error(mq135_data);
    }
    vTaskDelay(mq135_polling_rate_ticks);
  }
}


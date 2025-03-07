/* components/sensors/bh1750_hal/bh1750_hal.c */

#include "bh1750_hal.h"
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "error_handler.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint8_t    bh1750_i2c_address            = 0x23;
const i2c_port_t bh1750_i2c_bus                = I2C_NUM_0;
const char      *bh1750_tag                    = "BH1750";
const uint8_t    bh1750_scl_io                 = GPIO_NUM_22;
const uint8_t    bh1750_sda_io                 = GPIO_NUM_21;
const uint32_t   bh1750_i2c_freq_hz            = 100000;
const uint32_t   bh1750_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t    bh1750_max_retries            = 4;
const uint32_t   bh1750_initial_retry_interval = pdMS_TO_TICKS(15);
const uint32_t   bh1750_max_backoff_interval   = pdMS_TO_TICKS(8 * 60);
const uint8_t    bh1750_measurement_bytes      = 2;
const float      bh1750_raw_to_lux_factor      = 1.2f;
const uint8_t    bh1750_high_byte_shift        = 8;

/* Static Functions **********************************************************/

/**
 * @brief Reset function for the BH1750 sensor
 * 
 * This function is called by the error handler when a reset is needed.
 * It performs a full reinitialization of the sensor.
 * 
 * @param context Pointer to the bh1750_data_t structure
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t priv_bh1750_reset(void *context)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)context;
  esp_err_t      ret;

  /* Power on the sensor */
  ret = priv_i2c_write_byte(k_bh1750_power_on_cmd, 
                            bh1750_i2c_bus,
                            bh1750_i2c_address, 
                            bh1750_tag);
  if (ret != ESP_OK) {
    bh1750_data->state = k_bh1750_power_on_error;
    log_error(bh1750_tag, "Power Error", "Failed to power on BH1750 sensor");
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Reset the sensor */
  ret = priv_i2c_write_byte(k_bh1750_reset_cmd, 
                            bh1750_i2c_bus,
                            bh1750_i2c_address, 
                            bh1750_tag);
  if (ret != ESP_OK) {
    bh1750_data->state = k_bh1750_reset_error;
    log_error(bh1750_tag, "Reset Error", "Failed to reset BH1750 sensor");
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Set continuous measurement mode (low res) */
  ret = priv_i2c_write_byte(k_bh1750_cont_low_res_mode_cmd, 
                            bh1750_i2c_bus,
                            bh1750_i2c_address, 
                            bh1750_tag);
  if (ret != ESP_OK) {
    bh1750_data->state = k_bh1750_cont_low_res_error;
    log_error(bh1750_tag, 
              "Mode Error", 
              "Failed to set continuous low resolution mode");
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  bh1750_data->state = k_bh1750_ready;
  return ESP_OK;
}

/* Public Functions ***********************************************************/

char *bh1750_data_to_json(const bh1750_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    log_error(bh1750_tag, 
              "JSON Error", 
              "Failed to allocate memory for JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "light")) {
    log_error(bh1750_tag, 
              "JSON Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "lux", data->lux)) {
    log_error(bh1750_tag, 
              "JSON Error", 
              "Failed to add lux field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(bh1750_tag, 
              "JSON Error", 
              "Failed to serialize JSON object to string");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t bh1750_init(void *sensor_data)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)sensor_data;
  log_info(bh1750_tag, "Init Start", "Beginning BH1750 sensor initialization");

  /* Initialize error handler */
  error_handler_init(&(bh1750_data->error_handler), 
                     bh1750_tag,
                     bh1750_max_retries, 
                     bh1750_initial_retry_interval,
                     bh1750_max_backoff_interval, 
                     priv_bh1750_reset,
                     bh1750_data, 
                     bh1750_initial_retry_interval,
                     bh1750_max_backoff_interval);

  bh1750_data->i2c_address = bh1750_i2c_address;
  bh1750_data->i2c_bus     = bh1750_i2c_bus;
  bh1750_data->lux         = -1.0f;
  bh1750_data->state       = k_bh1750_uninitialized;

  /* Initialize the I2C bus */
  esp_err_t ret = priv_i2c_init(bh1750_scl_io, 
                                bh1750_sda_io, 
                                bh1750_i2c_freq_hz,
                                bh1750_i2c_bus, 
                                bh1750_tag);
  if (ret != ESP_OK) {
    log_error(bh1750_tag, "I2C Error", "Failed to initialize I2C driver");
    return ret;
  }

  /* Perform initial sensor setup */
  ret = priv_bh1750_reset(bh1750_data);
  if (ret != ESP_OK) {
    return ret;
  }

  log_info(bh1750_tag, "Init Complete", "BH1750 sensor initialized successfully");
  return ESP_OK;
}

esp_err_t bh1750_read(bh1750_data_t *sensor_data)
{
  /* Buffer to store the 2-byte measurement value from BH1750 */
  uint8_t   data[bh1750_measurement_bytes];
  esp_err_t ret = priv_i2c_read_bytes(data, 
                                      bh1750_measurement_bytes, 
                                      bh1750_i2c_bus,
                                      bh1750_i2c_address, 
                                      bh1750_tag);
  if (ret != ESP_OK) {
    sensor_data->lux   = -1.0;
    sensor_data->state = k_bh1750_error;
    log_error(bh1750_tag, 
              "Read Error", 
              "Failed to read light intensity data via I2C");
    return ESP_FAIL;
  }

  /* Combine high byte and low byte to form 16-bit measurement value */
  uint16_t raw_light_intensity = (data[0] << bh1750_high_byte_shift) | data[1];
  sensor_data->lux             = raw_light_intensity / bh1750_raw_to_lux_factor;
  log_info(bh1750_tag, 
           "Data Update", 
           "New reading - Light intensity: %.2f lux", 
           sensor_data->lux);

  sensor_data->state = k_bh1750_data_updated;
  return ESP_OK;
}

void bh1750_tasks(void *sensor_data)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)sensor_data;
  if (!bh1750_data) {
    log_error(bh1750_tag, "Task Error", "Invalid sensor data pointer provided");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    esp_err_t ret = bh1750_read(bh1750_data);
    if (ret == ESP_OK) {
      char *json = bh1750_data_to_json(bh1750_data);
      if (json) {
        send_sensor_data_to_webserver(json);
        file_write_enqueue("bh1750.txt", json);
        free(json);
      } else {
        log_error(bh1750_tag, 
                  "JSON Error", 
                  "Failed to convert sensor data to JSON format");
      }
    } else if (bh1750_data->state & k_bh1750_error) {
      error_handler_record_error(&(bh1750_data->error_handler), ESP_FAIL);
    }
    vTaskDelay(bh1750_polling_rate_ticks);
  }
}

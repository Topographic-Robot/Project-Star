/* components/sensors/qmc5883l_hal/qmc5883l_hal.c */

/* TODO: DRDY */

#include "qmc5883l_hal.h"
#include <math.h>
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const uint8_t    qmc5883l_i2c_address            = 0x0D;
const i2c_port_t qmc5883l_i2c_bus                = I2C_NUM_0;
const char      *qmc5883l_tag                    = "QMC5883L";
const uint8_t    qmc5883l_scl_io                 = GPIO_NUM_22;
const uint8_t    qmc5883l_sda_io                 = GPIO_NUM_21;
const uint32_t   qmc5883l_i2c_freq_hz            = 100000;
const uint32_t   qmc5883l_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t    qmc5883l_odr_setting            = k_qmc5883l_odr_100hz;
const uint8_t    qmc5883l_max_retries            = 4;
const uint32_t   qmc5883l_initial_retry_interval = pdMS_TO_TICKS(15);
const uint32_t   qmc5883l_max_backoff_interval   = pdMS_TO_TICKS(8 * 60);

/* Globals (Static) ***********************************************************/

static const qmc5883l_scale_t qmc5883l_scale_configs[] = {
  {k_qmc5883l_range_2g, 200.0 / 32768.0 }, /**< ±2 Gauss range, scaling factor */
  {k_qmc5883l_range_8g, 800.0 / 32768.0 }, /**< ±8 Gauss range, scaling factor */
};

static const uint8_t qmc5883l_scale_config_idx = 0; /**< Index of chosen values (0 for ±2G, 1 for ±8G) */

/* Public Functions ***********************************************************/

char *qmc5883l_data_to_json(const qmc5883l_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    ESP_LOGE(qmc5883l_tag, "Failed to create JSON object.");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "magnetometer")) {
    ESP_LOGE(qmc5883l_tag, "Failed to add sensor_type to JSON.");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_x", data->mag_x)) {
    ESP_LOGE(qmc5883l_tag, "Failed to add mag_x to JSON.");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_y", data->mag_y)) {
    ESP_LOGE(qmc5883l_tag, "Failed to add mag_y to JSON.");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_z", data->mag_z)) {
    ESP_LOGE(qmc5883l_tag, "Failed to add mag_z to JSON.");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "heading", data->heading)) {
    ESP_LOGE(qmc5883l_tag, "Failed to add heading to JSON.");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    ESP_LOGE(qmc5883l_tag, "Failed to serialize JSON object.");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t qmc5883l_init(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  ESP_LOGI(qmc5883l_tag, "Starting Configuration");

  qmc5883l_data->i2c_address        = qmc5883l_i2c_address;
  qmc5883l_data->i2c_bus            = qmc5883l_i2c_bus;
  qmc5883l_data->mag_x              = qmc5883l_data->mag_y = qmc5883l_data->mag_z = 0.0;
  qmc5883l_data->state              = k_qmc5883l_uninitialized;
  qmc5883l_data->retry_count        = 0;
  qmc5883l_data->retry_interval     = qmc5883l_initial_retry_interval;
  qmc5883l_data->last_attempt_ticks = 0;

  esp_err_t ret = priv_i2c_init(qmc5883l_scl_io, qmc5883l_sda_io,
                                qmc5883l_i2c_freq_hz, qmc5883l_i2c_bus,
                                qmc5883l_tag);

  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    qmc5883l_data->state = k_qmc5883l_power_on_error;
    return ret;
  }

  uint8_t ctrl1 = k_qmc5883l_mode_continuous | qmc5883l_odr_setting |
                  qmc5883l_scale_configs[qmc5883l_scale_config_idx].range |
                  k_qmc5883l_osr_512;

  ret = priv_i2c_write_reg_byte(k_qmc5883l_ctrl1_cmd, ctrl1,
                                qmc5883l_i2c_bus, qmc5883l_i2c_address,
                                qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "Configuration of CTRL1 register failed");
    qmc5883l_data->state = k_qmc5883l_error;
    return ret;
  }

  qmc5883l_data->state = k_qmc5883l_ready;
  ESP_LOGI(qmc5883l_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

esp_err_t qmc5883l_read(qmc5883l_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(qmc5883l_tag, "Sensor data pointer is NULL");
    return ESP_FAIL;
  }

  uint8_t   mag_data[6]; /* TODO: Move 6 to an enum or something */
  esp_err_t ret = priv_i2c_read_reg_bytes(0x00, mag_data, 6, /* Move 0x00 and 6 to enums or something */
                                          sensor_data->i2c_bus,
                                          sensor_data->i2c_address,
                                          qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "Failed to read magnetometer data from QMC5883L");
    sensor_data->state = k_qmc5883l_error;
    return ESP_FAIL;
  }

  sensor_data->state = k_qmc5883l_data_updated;

  int16_t mag_x_raw = (int16_t)((mag_data[1] << 8) | mag_data[0]); /* TODO: Add a comment to explain the index values and 8 */
  int16_t mag_y_raw = (int16_t)((mag_data[3] << 8) | mag_data[2]); /* TODO: Add a comment to explain the index values and 8 */
  int16_t mag_z_raw = (int16_t)((mag_data[5] << 8) | mag_data[4]); /* TODO: Add a comment to explain the index values and 8 */

  float scale_factor = qmc5883l_scale_configs[qmc5883l_scale_config_idx].scale;
  sensor_data->mag_x = mag_x_raw * scale_factor;
  sensor_data->mag_y = mag_y_raw * scale_factor;
  sensor_data->mag_z = mag_z_raw * scale_factor;

  float heading = atan2(sensor_data->mag_y, sensor_data->mag_x) * 180.0 / M_PI;
  if (heading < 0) {
    heading += 360.0;
  }
  sensor_data->heading = heading;

  ESP_LOGI(qmc5883l_tag, "Mag X: %f, Mag Y: %f, Mag Z: %f, Heading: %f degrees",
           sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z,
           sensor_data->heading);
  sensor_data->state = k_qmc5883l_data_updated;
  return ESP_OK;
}

void qmc5883l_reset_on_error(qmc5883l_data_t *sensor_data)
{
  if (sensor_data->state & k_qmc5883l_error) {
    TickType_t now_ticks = xTaskGetTickCount();
    if (now_ticks - sensor_data->last_attempt_ticks >= sensor_data->retry_interval) {
      sensor_data->last_attempt_ticks = now_ticks;

      if (qmc5883l_init(sensor_data) == ESP_OK) {
        sensor_data->state          = k_qmc5883l_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = qmc5883l_initial_retry_interval;
      } else {
        sensor_data->retry_count += 1;

        if (sensor_data->retry_count >= qmc5883l_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 <= qmc5883l_max_backoff_interval) ?
                                        sensor_data->retry_interval * 2 :
                                        qmc5883l_max_backoff_interval;
        }
      }
    }
  }
}

void qmc5883l_tasks(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  while (1) {
    if (qmc5883l_read(qmc5883l_data) == ESP_OK) {
      char *json = qmc5883l_data_to_json(qmc5883l_data);
      send_sensor_data_to_webserver(json);
      file_write_enqueue("qmc5883l.txt", json);
      free(json);
    } else {
      qmc5883l_reset_on_error(qmc5883l_data);
    }
    vTaskDelay(qmc5883l_polling_rate_ticks);
  }
}

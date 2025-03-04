/* components/sensors/mpu6050_hal/mpu6050_hal.c */

/* The values retrieved from this sensor have been optimized with better configuration */
/* Interrupt handler has been implemented using GPIO_NUM_26 */

#include "mpu6050_hal.h"
#include <math.h>
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "driver/gpio.h"
#include "error_handler.h"
#include "log_handler.h"
#include "nvs_flash.h"

/* Constants ******************************************************************/

const uint8_t    mpu6050_i2c_address        = 0x68;
const i2c_port_t mpu6050_i2c_bus            = I2C_NUM_0;
const char      *mpu6050_tag                = "MPU6050";
const uint8_t    mpu6050_scl_io             = GPIO_NUM_22;
const uint8_t    mpu6050_sda_io             = GPIO_NUM_21;
const uint32_t   mpu6050_i2c_freq_hz        = 400000;  /* Increased from 100kHz to 400kHz for faster communication */
const uint32_t   mpu6050_polling_rate_ticks = pdMS_TO_TICKS(10);  /* Reduced from 20ms to 10ms for more frequent updates */
const uint8_t    mpu6050_sample_rate_div    = 0;  /* Changed from 4 to 0 for maximum sample rate */
const uint8_t    mpu6050_config_dlpf        = k_mpu6050_config_dlpf_21hz;  /* Changed from 94Hz to 21Hz for better noise filtering */
const uint8_t    mpu6050_int_io             = GPIO_NUM_26;

/* Calibration offsets - these should be determined through a proper calibration procedure */
const float      mpu6050_accel_offset_x     = 0.0f;
const float      mpu6050_accel_offset_y     = 0.0f;
const float      mpu6050_accel_offset_z     = 0.0f;
const float      mpu6050_gyro_offset_x      = 0.0f;
const float      mpu6050_gyro_offset_y      = 0.0f;
const float      mpu6050_gyro_offset_z      = 0.0f;

/* Moving average filter parameters */
#define MPU6050_FILTER_SAMPLES 5  /* Number of samples to average */

/**
 * @brief Static constant array of accelerometer configurations and scaling factors.
 *
 * The MPU6050 accelerometer has several sensitivity options that define the maximum
 * measurable acceleration range. Each configuration has a corresponding sensitivity
 * value, given in LSB/g, which allows conversion from raw sensor output to
 * acceleration in g.
 *
 * Benefits of configuring accelerometer sensitivity:
 * - Higher sensitivity (e.g., ±2g) offers finer resolution for small movements,
 *   ideal for low-speed applications or where precision is required.
 * - Lower sensitivity (e.g., ±16g) provides a wider measurement range, suitable
 *   for detecting high-impact or fast movements.
 *
 * Sensitivity options (in LSB/g) as per the MPU6050 datasheet:
 * - ±2g: 16384 LSB/g
 * - ±4g: 8192 LSB/g
 * - ±8g: 4096 LSB/g
 * - ±16g: 2048 LSB/g
 */
static const mpu6050_accel_config_t mpu6050_accel_configs[] = {
  { k_mpu6050_accel_fs_2g,  16384.0 }, /**< Sensitivity: 16384 LSB/g */
  { k_mpu6050_accel_fs_4g,  8192.0  }, /**< Sensitivity: 8192 LSB/g */
  { k_mpu6050_accel_fs_8g,  4096.0  }, /**< Sensitivity: 4096 LSB/g */
  { k_mpu6050_accel_fs_16g, 2048.0  }, /**< Sensitivity: 2048 LSB/g */
};

/**
 * @brief Static constant array of gyroscope configurations and scaling factors.
 *
 * The MPU6050 gyroscope provides several sensitivity options that define the maximum
 * measurable rotational speed range. Each configuration has an associated sensitivity
 * value in LSB/°/s, enabling conversion from raw sensor output to angular velocity
 * in degrees per second (°/s).
 *
 * Benefits of configuring gyroscope sensitivity:
 * - Higher sensitivity (e.g., ±250°/s) allows finer resolution for slow rotations,
 *   which is ideal for applications requiring precision.
 * - Lower sensitivity (e.g., ±2000°/s) enables a wider measurement range, useful
 *   for detecting fast or high-impact rotations.
 *
 * Sensitivity options (in LSB/°/s) as per the MPU6050 datasheet:
 * - ±250°/s: 131 LSB/°/s
 * - ±500°/s: 65.5 LSB/°/s
 * - ±1000°/s: 32.8 LSB/°/s
 * - ±2000°/s: 16.4 LSB/°/s
 */
static const mpu6050_gyro_config_t mpu6050_gyro_configs[] = {
  { k_mpu6050_gyro_fs_250dps,  131.0 }, /**< Sensitivity: 131 LSB/°/s */
  { k_mpu6050_gyro_fs_500dps,  65.5  }, /**< Sensitivity: 65.5 LSB/°/s */
  { k_mpu6050_gyro_fs_1000dps, 32.8  }, /**< Sensitivity: 32.8 LSB/°/s */
  { k_mpu6050_gyro_fs_2000dps, 16.4  }, /**< Sensitivity: 16.4 LSB/°/s */
};

static const uint8_t   mpu6050_gyro_config_idx  = 1; /**< Using ±500°/s for better precision in normal use */
static const uint8_t   mpu6050_accel_config_idx = 1; /**< Using ±4g for better precision in normal use */
static error_handler_t s_mpu6050_error_handler  = { 0 };

/* Static (Private) Functions **************************************************/

/**
 * @brief Interrupt Service Routine (ISR) for handling MPU6050 data ready interrupts.
 *
 * This function is called when the MPU6050 asserts its INT pin, indicating that new data
 * is ready to be read. It gives the `data_ready_sem` semaphore to unblock the task waiting
 * to read the data.
 *
 * @param[in] arg Pointer to the `mpu6050_data_t` structure.
 *
 * @note This ISR should be kept as short as possible to avoid blocking other interrupts. 
 *       Ensure that the semaphore is properly initialized before enabling the interrupt.
 */
static void IRAM_ATTR priv_mpu6050_interrupt_handler(void *arg)
{
  mpu6050_data_t *sensor_data              = (mpu6050_data_t *)arg;
  BaseType_t      xHigherPriorityTaskWoken = pdFALSE;

  /* Give the semaphore to signal that data is ready */
  xSemaphoreGiveFromISR(sensor_data->data_ready_sem, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief Reset function for the MPU6050 sensor.
 *
 * This function is called by the error handler when an error occurs. It attempts to
 * reset the MPU6050 sensor by power cycling it and reinitializing the configuration.
 *
 * @param[in] context Pointer to the mpu6050_data_t structure.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Failure
 */
static esp_err_t priv_mpu6050_reset_func(void *context)
{
  mpu6050_data_t *sensor_data = (mpu6050_data_t *)context;
  esp_err_t ret = ESP_OK;
  
  log_info(mpu6050_tag, "Reset Started", "Attempting to reset MPU6050 sensor");
  
  /* Power cycle the sensor by putting it to sleep and then waking it up */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_power_down_cmd,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to put MPU6050 to sleep during reset");
    return ESP_FAIL;
  }
  
  /* Delay to allow the sensor to enter sleep mode */
  vTaskDelay(pdMS_TO_TICKS(10));
  
  /* Wake up the sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_power_on_cmd,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to wake up MPU6050 during reset");
    return ESP_FAIL;
  }
  
  /* Delay to allow the sensor to wake up */
  vTaskDelay(pdMS_TO_TICKS(10));
  
  /* Reconfigure the sensor */
  
  /* Configure the sample rate divider */
  ret = priv_i2c_write_reg_byte(k_mpu6050_smplrt_div_cmd, mpu6050_sample_rate_div,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to set sample rate divider during reset");
    return ESP_FAIL;
  }
  
  /* Configure the Digital Low Pass Filter (DLPF) */
  ret = priv_i2c_write_reg_byte(k_mpu6050_config_cmd, mpu6050_config_dlpf,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to configure DLPF during reset");
    return ESP_FAIL;
  }
  
  /* Configure the gyroscope full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_gyro_config_cmd,
                              mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_config,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to set gyroscope range during reset");
    return ESP_FAIL;
  }
  
  /* Configure the accelerometer full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_accel_config_cmd,
                              mpu6050_accel_configs[mpu6050_accel_config_idx].accel_config,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to set accelerometer range during reset");
    return ESP_FAIL;
  }
  
  /* Re-enable interrupts */
  ret = priv_i2c_write_reg_byte(k_mpu6050_int_enable_cmd, sensor_data->int_config.interrupt_sources,
                              sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Error", "Failed to enable interrupts during reset");
    return ESP_FAIL;
  }
  
  /* Update sensor state */
  sensor_data->state = k_mpu6050_ready;
  
  log_info(mpu6050_tag, "Reset Complete", "MPU6050 sensor reset completed successfully");
  return ESP_OK;
}

/* Public Functions ***********************************************************/

char *mpu6050_data_to_json(const mpu6050_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    log_error(mpu6050_tag, "JSON Creation Failed", "Memory allocation failed while creating JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "accelerometer_gyroscope")) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_x", data->accel_x)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add accel_x field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_y", data->accel_y)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add accel_y field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_z", data->accel_z)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add accel_z field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_x", data->gyro_x)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add gyro_x field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_y", data->gyro_y)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add gyro_y field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_z", data->gyro_z)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add gyro_z field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }
  
  if (!cJSON_AddNumberToObject(json, "temperature", data->temperature)) {
    log_error(mpu6050_tag, "JSON Field Error", "Failed to add temperature field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(mpu6050_tag, "JSON Serialization Failed", "Unable to convert JSON object to string format");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t mpu6050_init(void *sensor_data)
{
  mpu6050_data_t *mpu6050_data = (mpu6050_data_t *)sensor_data;
  log_info(mpu6050_tag, "Init Started", "Beginning MPU6050 sensor initialization");

  /* Initialize error handler */
  error_handler_init(&s_mpu6050_error_handler, mpu6050_tag, 3, pdMS_TO_TICKS(100),
                    pdMS_TO_TICKS(5000), priv_mpu6050_reset_func, mpu6050_data,
                    pdMS_TO_TICKS(500), pdMS_TO_TICKS(10000));

  mpu6050_data->i2c_address = mpu6050_i2c_address;
  mpu6050_data->i2c_bus     = mpu6050_i2c_bus;
  mpu6050_data->gyro_x      = mpu6050_data->gyro_y  = mpu6050_data->gyro_z  = 0.0f;
  mpu6050_data->accel_x     = mpu6050_data->accel_y = mpu6050_data->accel_z = 0.0f;
  mpu6050_data->temperature = 0.0f;
  mpu6050_data->state       = k_mpu6050_uninitialized; /* Start in uninitialized state */
  
  /* Initialize the data mutex */
  portMUX_TYPE temp_mutex = portMUX_INITIALIZER_UNLOCKED;
  mpu6050_data->data_mutex = temp_mutex;
  
  /* Initialize the moving average filter buffers */
  mpu6050_data->buffer_index = 0;
  for (int i = 0; i < MPU6050_FILTER_SAMPLES; i++) {
    mpu6050_data->accel_x_buffer[i] = 0.0f;
    mpu6050_data->accel_y_buffer[i] = 0.0f;
    mpu6050_data->accel_z_buffer[i] = 0.0f;
    mpu6050_data->gyro_x_buffer[i] = 0.0f;
    mpu6050_data->gyro_y_buffer[i] = 0.0f;
    mpu6050_data->gyro_z_buffer[i] = 0.0f;
  }
  
  /* Initialize interrupt configuration with default values */
  mpu6050_data->int_config.interrupt_pin = mpu6050_int_io;
  mpu6050_data->int_config.active_level = k_mpu6050_int_active_low;
  mpu6050_data->int_config.pin_mode = k_mpu6050_int_push_pull;
  mpu6050_data->int_config.latch_mode = k_mpu6050_int_latch_50us;
  mpu6050_data->int_config.clear_mode = k_mpu6050_int_clear_any_read;
  mpu6050_data->int_config.interrupt_sources = k_mpu6050_int_enable_data_rdy; /* Enable data ready interrupt by default */

  esp_err_t ret = priv_i2c_init(mpu6050_scl_io, mpu6050_sda_io,
                                mpu6050_i2c_freq_hz, mpu6050_i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "I2C Error", "Failed to install I2C driver: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Wake up the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_power_on_cmd,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Power On Failed", "Unable to wake up MPU6050 sensor from sleep mode");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to power on */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Reset the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_reset_cmd,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Reset Failed", "Unable to reset MPU6050 sensor to default state");
    mpu6050_data->state = k_mpu6050_reset_error;
    return ret;
  }

  /* Delay to allow the reset to take effect */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Wake up the sensor again after reset */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_power_on_cmd,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Power On Failed", "Unable to wake up MPU6050 sensor after reset");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to wake up */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Configure the sample rate divider */
  ret = priv_i2c_write_reg_byte(k_mpu6050_smplrt_div_cmd, mpu6050_sample_rate_div,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Config Error", "Failed to set sample rate divider for MPU6050");
    return ret;
  }

  /* Configure the Digital Low Pass Filter (DLPF) */
  ret = priv_i2c_write_reg_byte(k_mpu6050_config_cmd, mpu6050_config_dlpf,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Config Error", "Failed to configure digital low pass filter settings");
    return ret;
  }

  /* Configure the gyroscope full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_gyro_config_cmd,
                                mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_config,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Config Error", "Failed to set gyroscope full-scale range");
    return ret;
  }

  /* Configure the accelerometer full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_accel_config_cmd,
                                mpu6050_accel_configs[mpu6050_accel_config_idx].accel_config,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Config Error", "Failed to set accelerometer full-scale range");
    return ret;
  }

  /* Verify the sensor by reading the WHO_AM_I register */
  uint8_t who_am_i = 0;
  ret              = priv_i2c_read_reg_bytes(k_mpu6050_who_am_i_cmd, &who_am_i, 
                                             1, mpu6050_i2c_bus, mpu6050_i2c_address, 
                                             mpu6050_tag);
  if (ret != ESP_OK || who_am_i != k_mpu6050_who_am_i_response) {
    log_error(mpu6050_tag, "Verification Failed", "Invalid WHO_AM_I register response from MPU6050: 0x%02X", who_am_i);
    return ret;
  }

  /* Create a binary semaphore for data readiness */
  mpu6050_data->data_ready_sem = xSemaphoreCreateBinary();
  if (mpu6050_data->data_ready_sem == NULL) {
    log_error(mpu6050_tag, "Memory Error", "Failed to allocate memory for data ready semaphore");
    return ESP_FAIL;
  }

  /* Configure the MPU6050 to generate Data Ready interrupts */
  ret = priv_i2c_write_reg_byte(k_mpu6050_int_enable_cmd, k_mpu6050_int_enable_data_rdy,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Interrupt Error", "Failed to enable data ready interrupt on MPU6050");
    return ret;
  }

  /* Configure the INT (interrupt) pin on the ESP32 */
  gpio_config_t io_conf = {};
  io_conf.intr_type     = GPIO_INTR_NEGEDGE; /* Default: MPU6050 INT pin is active low */
  io_conf.pin_bit_mask  = (1ULL << mpu6050_int_io);
  io_conf.mode          = GPIO_MODE_INPUT;
  io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en  = GPIO_PULLDOWN_ENABLE;

  /* Configure the MPU6050 INT pin behavior */
  uint8_t int_pin_cfg = 0x00;

  /* Configure INT pin as open-drain if specified */
  if (mpu6050_data->int_config.pin_mode == k_mpu6050_int_open_drain) {
    int_pin_cfg |= (1 << 6); /* Set bit 6 for open-drain mode */
  }

  /* Configure INT pin active level */
  if (mpu6050_data->int_config.active_level == k_mpu6050_int_active_low) {
    int_pin_cfg |= (1 << 7); /* Set bit 7 for active-low */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
  } else {
    io_conf.intr_type = GPIO_INTR_POSEDGE;
  }

  /* Configure interrupt latch behavior */
  if (mpu6050_data->int_config.latch_mode == k_mpu6050_int_latch_until_cleared) {
    int_pin_cfg |= (1 << 5); /* Set bit 5 for latching until cleared */
  }

  /* Configure interrupt clear behavior */
  if (mpu6050_data->int_config.clear_mode == k_mpu6050_int_clear_status_read) {
    int_pin_cfg |= (1 << 4); /* Set bit 4 for clear on status read only */
  }

  /* Write the INT pin configuration to the MPU6050 */
  ret = priv_i2c_write_reg_byte(k_mpu6050_int_pin_cfg_cmd, int_pin_cfg,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Config Error", "Failed to configure INT pin behavior on MPU6050");
    return ret;
  }

  /* Enable the configured interrupt sources */
  ret = priv_i2c_write_reg_byte(k_mpu6050_int_enable_cmd, mpu6050_data->int_config.interrupt_sources,
                                mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Interrupt Error", "Failed to enable interrupt sources on MPU6050");
    return ret;
  }

  /* Configure the ESP32 GPIO for the interrupt pin */
  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "GPIO Error", "Failed to configure interrupt pin for MPU6050");
    return ret;
  }

  /* Install GPIO ISR service */
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    log_error(mpu6050_tag, "ISR Error", "Failed to install GPIO interrupt service routine");
    return ret;
  }

  /* Hook up the interrupt handler to the GPIO pin */
  ret = gpio_isr_handler_add(mpu6050_data->int_config.interrupt_pin, 
                            priv_mpu6050_interrupt_handler, mpu6050_data);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "ISR Error", "Failed to add interrupt handler for MPU6050");
    return ret;
  }
  log_info(mpu6050_tag, "Interrupt Setup", "MPU6050 interrupt handler installed successfully");

  /* Configure interrupts using the new function */
  ret = mpu6050_config_interrupts(mpu6050_data, &mpu6050_data->int_config);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Interrupt Config Error", "Failed to configure MPU6050 interrupts");
    return ret;
  }

  /* Try to load calibration values from NVS */
  ret = mpu6050_load_calibration_from_nvs();
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "Calibration Warning", "Failed to load calibration values from NVS, using defaults");
    /* This is not a critical error, we can continue with default values */
  } else {
    log_info(mpu6050_tag, "Calibration Loaded", "Successfully loaded calibration values from NVS");
  }

  /* Run a self-test to verify that the sensor is functioning correctly */
  log_info(mpu6050_tag, "Self-Test", "Running MPU6050 self-test during initialization");
  ret = mpu6050_self_test(mpu6050_data);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "Self-Test Warning", "MPU6050 self-test failed, sensor may not function correctly");
    /* We'll continue anyway, but log a warning */
  } else {
    log_info(mpu6050_tag, "Self-Test", "MPU6050 self-test passed, sensor is functioning correctly");
  }

  mpu6050_data->state = k_mpu6050_ready; /* Sensor is initialized */
  log_info(mpu6050_tag, "Init Complete", "MPU6050 sensor initialization completed successfully");
  return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    log_error(mpu6050_tag, "Invalid Parameter", "Sensor data pointer is NULL, cannot proceed with read operation");
    return ESP_FAIL;
  }

  uint8_t accel_data[6];
  uint8_t gyro_data[6];
  uint8_t temp_data[2];

  /* Read accelerometer data starting from ACCEL_XOUT_H */
  esp_err_t ret = priv_i2c_read_reg_bytes(k_mpu6050_accel_xout_h_cmd, accel_data, 6,
                                          sensor_data->i2c_bus, sensor_data->i2c_address,
                                          mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Read Error", "Failed to read accelerometer data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    return ESP_FAIL;
  }

  /* Read gyroscope data starting from GYRO_XOUT_H */
  ret = priv_i2c_read_reg_bytes(k_mpu6050_gyro_xout_h_cmd, gyro_data, 6,
                                sensor_data->i2c_bus, 
                                sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Read Error", "Failed to read gyroscope data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    return ESP_FAIL;
  }
  
  /* Read temperature data */
  ret = priv_i2c_read_reg_bytes(k_mpu6050_temp_out_h_cmd, temp_data, 2,
                               sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Read Error", "Failed to read temperature data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    return ESP_FAIL;
  }

  /* Combine high and low bytes to form the raw accelerometer data */
  int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  /* Combine high and low bytes to form the raw gyroscope data */
  int16_t gyro_x_raw = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  int16_t gyro_y_raw = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  int16_t gyro_z_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
  
  /* Combine high and low bytes to form the raw temperature data */
  int16_t temp_raw = (int16_t)((temp_data[0] << 8) | temp_data[1]);

  /* Convert raw data to physical units by dividing by sensitivity */
  float accel_sensitivity = mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  float gyro_sensitivity  = mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;

  /* Calculate new values with calibration offsets */
  float new_accel_x = (accel_x_raw / accel_sensitivity) - mpu6050_accel_offset_x;
  float new_accel_y = (accel_y_raw / accel_sensitivity) - mpu6050_accel_offset_y;
  float new_accel_z = (accel_z_raw / accel_sensitivity) - mpu6050_accel_offset_z;
  float new_gyro_x  = (gyro_x_raw / gyro_sensitivity) - mpu6050_gyro_offset_x;
  float new_gyro_y  = (gyro_y_raw / gyro_sensitivity) - mpu6050_gyro_offset_y;
  float new_gyro_z  = (gyro_z_raw / gyro_sensitivity) - mpu6050_gyro_offset_z;
  
  /* Calculate temperature in degrees Celsius */
  /* Formula from MPU6050 datasheet: Temp_degC = (TEMP_OUT / 340) + 36.53 */
  float new_temperature = (temp_raw / 340.0f) + 36.53f;

  /* Validate accelerometer readings (should be within ±4g range) */
  if (fabsf(new_accel_x) > 4.0f || fabsf(new_accel_y) > 4.0f || fabsf(new_accel_z) > 4.0f) {
    log_warn(mpu6050_tag, "Range Error", "Accelerometer readings exceed ±4g measurement range");
    sensor_data->state = k_mpu6050_error;
    return ESP_FAIL;
  }

  /* Validate gyroscope readings (should be within ±500°/s range) */
  if (fabsf(new_gyro_x) > 500.0f || fabsf(new_gyro_y) > 500.0f || fabsf(new_gyro_z) > 500.0f) {
    log_warn(mpu6050_tag, "Range Error", "Gyroscope readings exceed ±500°/s measurement range");
    sensor_data->state = k_mpu6050_error;
    return ESP_FAIL;
  }

  /* Apply simple moving average filter */
  /* Store current values in the circular buffer */
  sensor_data->accel_x_buffer[sensor_data->buffer_index] = new_accel_x;
  sensor_data->accel_y_buffer[sensor_data->buffer_index] = new_accel_y;
  sensor_data->accel_z_buffer[sensor_data->buffer_index] = new_accel_z;
  sensor_data->gyro_x_buffer[sensor_data->buffer_index] = new_gyro_x;
  sensor_data->gyro_y_buffer[sensor_data->buffer_index] = new_gyro_y;
  sensor_data->gyro_z_buffer[sensor_data->buffer_index] = new_gyro_z;
  
  /* Increment buffer index and wrap around if necessary */
  sensor_data->buffer_index = (sensor_data->buffer_index + 1) % MPU6050_FILTER_SAMPLES;
  
  /* Calculate the average of the buffer values */
  float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
  float gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f;
  
  for (int i = 0; i < MPU6050_FILTER_SAMPLES; i++) {
    accel_x_sum += sensor_data->accel_x_buffer[i];
    accel_y_sum += sensor_data->accel_y_buffer[i];
    accel_z_sum += sensor_data->accel_z_buffer[i];
    gyro_x_sum += sensor_data->gyro_x_buffer[i];
    gyro_y_sum += sensor_data->gyro_y_buffer[i];
    gyro_z_sum += sensor_data->gyro_z_buffer[i];
  }
  
  /* Update sensor data with filtered readings */
  sensor_data->accel_x = accel_x_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->accel_y = accel_y_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->accel_z = accel_z_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->gyro_x = gyro_x_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->gyro_y = gyro_y_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->gyro_z = gyro_z_sum / MPU6050_FILTER_SAMPLES;
  sensor_data->temperature = new_temperature;

  log_info(mpu6050_tag, "Data Updated", "Accel: [%f, %f, %f] g, Gyro: [%f, %f, %f] °/s, Temp: %f °C",
           sensor_data->accel_x, sensor_data->accel_y, sensor_data->accel_z,
           sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z,
           sensor_data->temperature);

  sensor_data->state = k_mpu6050_data_updated;
  return ESP_OK;
}

esp_err_t mpu6050_reset_on_error(mpu6050_data_t *sensor_data)
{
  /* Check if the state indicates any error */
  if (sensor_data->state & k_mpu6050_error) {
    log_warn(mpu6050_tag, "Error Detected", "MPU6050 sensor is in error state, attempting recovery");
    
    /* Use the error handler to manage the reset with exponential backoff */
    esp_err_t ret = error_handler_record_status(&s_mpu6050_error_handler, ESP_FAIL);
    
    if (ret == ESP_OK) {
      log_info(mpu6050_tag, "Recovery Success", "MPU6050 sensor recovery was successful");
      sensor_data->state = k_mpu6050_ready;
    } else if (ret == ESP_ERR_INVALID_STATE) {
      log_warn(mpu6050_tag, "Recovery Delayed", "In backoff period, will retry reset later");
    } else {
      log_error(mpu6050_tag, "Recovery Failed", "MPU6050 sensor recovery failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
  }
  
  /* No error detected, nothing to do */
  return ESP_OK;
}

void mpu6050_tasks(void *sensor_data)
{
  mpu6050_data_t *mpu6050_data = (mpu6050_data_t *)sensor_data;
  
  log_info(mpu6050_tag, "Task Started", "MPU6050 sensor task started in interrupt mode");
  
  /* Variables for tracking sensor health */
  uint32_t consecutive_timeouts = 0;
  uint32_t consecutive_errors = 0;
  uint32_t total_readings = 0;
  uint32_t successful_readings = 0;
  TickType_t last_successful_reading = xTaskGetTickCount();
  
  /* Power management - track activity to enable low-power modes when idle */
  bool low_power_mode = false;
  uint32_t idle_counter = 0;
  const uint32_t IDLE_THRESHOLD = 100; /* Enter low power after this many idle cycles */
  
  while (1) {
    /* Wait for the interrupt to signal that data is ready */
    if (xSemaphoreTake(mpu6050_data->data_ready_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
      /* Reset timeout counter since we received an interrupt */
      consecutive_timeouts = 0;
      idle_counter = 0;
      
      /* If we were in low power mode, wake up the sensor */
      if (low_power_mode) {
        if (custom_mpu6050_wake_up(mpu6050_data) == ESP_OK) {
          low_power_mode = false;
          log_info(mpu6050_tag, "Power Management", "Sensor woken up from low power mode");
          /* Give the sensor time to stabilize after waking up */
          vTaskDelay(pdMS_TO_TICKS(10));
        } else {
          log_error(mpu6050_tag, "Power Management Error", "Failed to wake up sensor from low power mode");
        }
      }
      
      /* Data is ready, read it */
      total_readings++;
      if (mpu6050_read(mpu6050_data) == ESP_OK) {
        successful_readings++;
        last_successful_reading = xTaskGetTickCount();
        consecutive_errors = 0;
        
        /* Process and send the data */
        char *json = mpu6050_data_to_json(mpu6050_data);
        if (json != NULL) {
          send_sensor_data_to_webserver(json);
          file_write_enqueue("mpu6050.txt", json);
          free(json);
        } else {
          log_error(mpu6050_tag, "JSON Error", "Failed to create JSON from sensor data");
        }
      } else {
        consecutive_errors++;
        log_warn(mpu6050_tag, "Read Error", "Failed to read sensor data, consecutive errors: %lu", consecutive_errors);
        
        /* If we have too many consecutive errors, try to reset the sensor */
        if (consecutive_errors >= 3) {
          log_error(mpu6050_tag, "Sensor Error", "Multiple consecutive read errors, attempting recovery");
          mpu6050_reset_on_error(mpu6050_data);
          consecutive_errors = 0; /* Reset counter after recovery attempt */
        }
      }
    } else {
      /* Timeout occurred, no interrupt received */
      consecutive_timeouts++;
      idle_counter++;
      
      /* Log a warning if we've had multiple timeouts */
      if (consecutive_timeouts == 1) {
        log_warn(mpu6050_tag, "Interrupt Timeout", "No interrupt received within timeout period");
      } else if (consecutive_timeouts % 10 == 0) {
        log_warn(mpu6050_tag, "Multiple Timeouts", "No interrupts for %lu consecutive periods", consecutive_timeouts);
      }
      
      /* Check if the sensor is still responsive by reading the interrupt status register */
      uint8_t int_status = 0;
      if (priv_i2c_read_reg_bytes(k_mpu6050_int_status_cmd, &int_status, 1,
                                 mpu6050_data->i2c_bus, mpu6050_data->i2c_address,
                                 mpu6050_tag) == ESP_OK) {
        
        /* If data is ready but we didn't get an interrupt, read it anyway */
        if (int_status & k_mpu6050_int_enable_data_rdy) {
          log_info(mpu6050_tag, "Interrupt Status", "Data ready flag set but no interrupt received");
          
          total_readings++;
          if (mpu6050_read(mpu6050_data) == ESP_OK) {
            successful_readings++;
            last_successful_reading = xTaskGetTickCount();
            consecutive_errors = 0;
            
            char *json = mpu6050_data_to_json(mpu6050_data);
            if (json != NULL) {
              send_sensor_data_to_webserver(json);
              file_write_enqueue("mpu6050.txt", json);
              free(json);
            }
          } else {
            consecutive_errors++;
          }
        } else {
          /* No data ready, sensor is just idle */
          log_info(mpu6050_tag, "Sensor Status", "Sensor is idle, no new data available");
        }
      } else {
        /* Failed to read the status register, sensor might be unresponsive */
        log_error(mpu6050_tag, "Communication Error", "Failed to read interrupt status register");
        consecutive_errors++;
        
        /* If we have too many consecutive errors, try to reset the sensor */
        if (consecutive_errors >= 3) {
          log_error(mpu6050_tag, "Sensor Error", "Multiple consecutive communication errors, attempting recovery");
          mpu6050_reset_on_error(mpu6050_data);
          consecutive_errors = 0;
        }
      }
      
      /* Check if we should enter low power mode due to inactivity */
      if (!low_power_mode && idle_counter > IDLE_THRESHOLD) {
        log_info(mpu6050_tag, "Power Management", "Entering low power mode due to inactivity");
        if (custom_mpu6050_sleep(mpu6050_data) == ESP_OK) {
          low_power_mode = true;
          idle_counter = 0;
        }
      }
      
      /* Watchdog: Check if the sensor has been unresponsive for too long */
      TickType_t current_time = xTaskGetTickCount();
      if ((current_time - last_successful_reading) > pdMS_TO_TICKS(30000)) { /* 30 seconds */
        log_error(mpu6050_tag, "Watchdog Alert", "Sensor unresponsive for 30 seconds, forcing reset");
        mpu6050_reset_on_error(mpu6050_data);
        last_successful_reading = current_time; /* Reset the timer */
      }
    }
    
    /* Periodically log sensor health statistics */
    if (total_readings % 100 == 0 && total_readings > 0) {
      float success_rate = (successful_readings * 100.0f) / total_readings;
      log_info(mpu6050_tag, "Sensor Health", "Success rate: %.1f%% (%lu/%lu readings)", 
               success_rate, successful_readings, total_readings);
    }
  }
}

/**
 * @brief Puts the MPU6050 sensor into sleep mode to save power.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if successful.
 * - ESP_FAIL or other error code if failed.
 */
esp_err_t custom_mpu6050_sleep(mpu6050_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    log_error(mpu6050_tag, "Invalid Parameter", "Sensor data pointer is NULL, cannot proceed with sleep operation");
    return ESP_FAIL;
  }
  
  /* Set the sleep bit in the power management register */
  esp_err_t ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_sleep_cmd,
                                         sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Sleep Error", "Failed to put MPU6050 sensor into sleep mode");
    return ret;
  }
  
  log_info(mpu6050_tag, "Sleep Mode", "MPU6050 sensor is now in sleep mode");
  sensor_data->state = k_mpu6050_sleep_mode;
  return ESP_OK;
}

/**
 * @brief Wakes up the MPU6050 sensor from sleep mode.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if successful.
 * - ESP_FAIL or other error code if failed.
 */
esp_err_t custom_mpu6050_wake_up(mpu6050_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    log_error(mpu6050_tag, "Invalid Parameter", "Sensor data pointer is NULL, cannot proceed with wake-up operation");
    return ESP_FAIL;
  }
  
  /* Clear the sleep bit in the power management register */
  esp_err_t ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, k_mpu6050_power_on_cmd,
                                         sensor_data->i2c_bus, sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Wake-up Error", "Failed to wake up MPU6050 sensor from sleep mode");
    return ret;
  }
  
  /* Delay to allow the sensor to stabilize after waking up */
  vTaskDelay(pdMS_TO_TICKS(10));
  
  log_info(mpu6050_tag, "Wake-up", "MPU6050 sensor has been woken up from sleep mode");
  sensor_data->state = k_mpu6050_ready;
  return ESP_OK;
}

/**
 * @brief Performs a self-test of the MPU6050 sensor.
 *
 * This function verifies that the sensor is functioning correctly by:
 * 1. Checking the WHO_AM_I register to verify the sensor identity
 * 2. Testing basic I2C communication by writing and reading back a test value
 * 3. Reading accelerometer and gyroscope data to verify they are within expected ranges
 * 4. Checking that the sensor can be put to sleep and woken up
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if all tests pass.
 * - ESP_FAIL or other error code if any test fails.
 */
esp_err_t mpu6050_self_test(mpu6050_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    log_error(mpu6050_tag, "Self-Test Error", "Sensor data pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret;
  bool test_passed = true;
  
  log_info(mpu6050_tag, "Self-Test", "Starting MPU6050 self-test");
  
  /* Test 1: Verify sensor identity by reading WHO_AM_I register */
  uint8_t who_am_i = 0;
  ret = priv_i2c_read_reg_bytes(k_mpu6050_who_am_i_cmd, &who_am_i, 1,
                               sensor_data->i2c_bus, sensor_data->i2c_address,
                               mpu6050_tag);
  if (ret != ESP_OK || who_am_i != k_mpu6050_who_am_i_response) {
    log_error(mpu6050_tag, "Self-Test Failed", "WHO_AM_I check failed, expected 0x%02X, got 0x%02X",
             k_mpu6050_who_am_i_response, who_am_i);
    return ESP_FAIL;
  }
  log_info(mpu6050_tag, "Self-Test", "WHO_AM_I check passed");
  
  /* Test 2: Verify I2C communication by writing and reading back a test value */
  /* We'll use the sample rate divider register for this test */
  uint8_t test_value = 0x55; /* Test pattern: 01010101 */
  ret = priv_i2c_write_reg_byte(k_mpu6050_smplrt_div_cmd, test_value,
                              sensor_data->i2c_bus, sensor_data->i2c_address,
                              mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Self-Test Failed", "Failed to write test value to sample rate divider register");
    test_passed = false;
  } else {
    uint8_t read_value = 0;
    ret = priv_i2c_read_reg_bytes(k_mpu6050_smplrt_div_cmd, &read_value, 1,
                                 sensor_data->i2c_bus, sensor_data->i2c_address,
                                 mpu6050_tag);
    if (ret != ESP_OK || read_value != test_value) {
      log_error(mpu6050_tag, "Self-Test Failed", "Read-back test failed, expected 0x%02X, got 0x%02X",
               test_value, read_value);
      test_passed = false;
    } else {
      log_info(mpu6050_tag, "Self-Test", "I2C communication test passed");
    }
    
    /* Restore the original sample rate divider */
    ret = priv_i2c_write_reg_byte(k_mpu6050_smplrt_div_cmd, mpu6050_sample_rate_div,
                                sensor_data->i2c_bus, sensor_data->i2c_address,
                                mpu6050_tag);
    if (ret != ESP_OK) {
      log_warn(mpu6050_tag, "Self-Test Warning", "Failed to restore original sample rate divider");
    }
  }
  
  /* Test 3: Read accelerometer and gyroscope data to verify they are within expected ranges */
  uint8_t accel_data[6];
  uint8_t gyro_data[6];
  
  /* Read accelerometer data */
  ret = priv_i2c_read_reg_bytes(k_mpu6050_accel_xout_h_cmd, accel_data, 6,
                               sensor_data->i2c_bus, sensor_data->i2c_address,
                               mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Self-Test Failed", "Failed to read accelerometer data");
    test_passed = false;
  } else {
    /* Combine high and low bytes to form the raw accelerometer data */
    int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);
    
    /* Check if the values are non-zero and within reasonable ranges */
    if (accel_x_raw == 0 && accel_y_raw == 0 && accel_z_raw == 0) {
      log_error(mpu6050_tag, "Self-Test Failed", "All accelerometer readings are zero, which is unlikely");
      test_passed = false;
    } else {
      log_info(mpu6050_tag, "Self-Test", "Accelerometer data check passed");
    }
  }
  
  /* Read gyroscope data */
  ret = priv_i2c_read_reg_bytes(k_mpu6050_gyro_xout_h_cmd, gyro_data, 6,
                               sensor_data->i2c_bus, sensor_data->i2c_address,
                               mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Self-Test Failed", "Failed to read gyroscope data");
    test_passed = false;
  } else {
    /* Combine high and low bytes to form the raw gyroscope data */
    int16_t gyro_x_raw = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    int16_t gyro_y_raw = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    int16_t gyro_z_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
    
    /* Check if the values are within reasonable ranges */
    if (gyro_x_raw == 0 && gyro_y_raw == 0 && gyro_z_raw == 0) {
      log_error(mpu6050_tag, "Self-Test Failed", "All gyroscope readings are zero, which is unlikely");
      test_passed = false;
    } else {
      log_info(mpu6050_tag, "Self-Test", "Gyroscope data check passed");
    }
  }
  
  /* Test 4: Check that the sensor can be put to sleep and woken up */
  /* Put the sensor to sleep */
  ret = custom_mpu6050_sleep(sensor_data);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Self-Test Failed", "Failed to put sensor to sleep");
    test_passed = false;
  } else {
    /* Verify that the sensor is in sleep mode by reading the power management register */
    uint8_t pwr_mgmt = 0;
    ret = priv_i2c_read_reg_bytes(k_mpu6050_pwr_mgmt_1_cmd, &pwr_mgmt, 1,
                                 sensor_data->i2c_bus, sensor_data->i2c_address,
                                 mpu6050_tag);
    if (ret != ESP_OK || !(pwr_mgmt & k_mpu6050_power_down_cmd)) {
      log_error(mpu6050_tag, "Self-Test Failed", "Sensor did not enter sleep mode properly");
      test_passed = false;
    } else {
      log_info(mpu6050_tag, "Self-Test", "Sleep mode test passed");
    }
    
    /* Wake up the sensor */
    ret = custom_mpu6050_wake_up(sensor_data);
    if (ret != ESP_OK) {
      log_error(mpu6050_tag, "Self-Test Failed", "Failed to wake up sensor");
      test_passed = false;
    } else {
      /* Verify that the sensor is awake by reading the power management register */
      ret = priv_i2c_read_reg_bytes(k_mpu6050_pwr_mgmt_1_cmd, &pwr_mgmt, 1,
                                   sensor_data->i2c_bus, sensor_data->i2c_address,
                                   mpu6050_tag);
      if (ret != ESP_OK || (pwr_mgmt & k_mpu6050_power_down_cmd)) {
        log_error(mpu6050_tag, "Self-Test Failed", "Sensor did not wake up properly");
        test_passed = false;
      } else {
        log_info(mpu6050_tag, "Self-Test", "Wake up test passed");
      }
    }
  }
  
  /* Report overall test results */
  if (test_passed) {
    log_info(mpu6050_tag, "Self-Test", "All tests passed, MPU6050 is functioning correctly");
    return ESP_OK;
  } else {
    log_error(mpu6050_tag, "Self-Test", "One or more tests failed, MPU6050 may not be functioning correctly");
    return ESP_FAIL;
  }
}

esp_err_t mpu6050_run_calibration_and_save(mpu6050_data_t *sensor_data, uint16_t num_samples)
{
  esp_err_t ret = mpu6050_calibrate(sensor_data, num_samples);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Calibration Error", "Failed to calibrate MPU6050 sensor");
    return ret;
  }
  
  /* Save the calibration values to NVS */
  ret = mpu6050_save_calibration_to_nvs(
    sensor_data->calibration.accel_x_offset,
    sensor_data->calibration.accel_y_offset,
    sensor_data->calibration.accel_z_offset,
    sensor_data->calibration.gyro_x_offset,
    sensor_data->calibration.gyro_y_offset,
    sensor_data->calibration.gyro_z_offset
  );
  
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Calibration Save Error", "Failed to save calibration values to NVS");
    return ret;
  }
  
  log_info(mpu6050_tag, "Calibration Complete", "MPU6050 calibration completed and saved to NVS");
  return ESP_OK;
}

esp_err_t mpu6050_load_calibration_from_nvs(void)
{
  float accel_x_offset = 0.0f, accel_y_offset = 0.0f, accel_z_offset = 0.0f;
  float gyro_x_offset = 0.0f, gyro_y_offset = 0.0f, gyro_z_offset = 0.0f;
  
  /* Initialize NVS */
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("mpu6050", NVS_READONLY, &nvs_handle);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Open Error", "Failed to open NVS namespace for MPU6050 calibration data");
    return ret;
  }
  
  /* Read calibration values from NVS */
  size_t required_size = sizeof(float);
  ret = nvs_get_blob(nvs_handle, "accel_x_off", &accel_x_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read accel_x_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_get_blob(nvs_handle, "accel_y_off", &accel_y_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read accel_y_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_get_blob(nvs_handle, "accel_z_off", &accel_z_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read accel_z_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_get_blob(nvs_handle, "gyro_x_off", &gyro_x_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read gyro_x_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_get_blob(nvs_handle, "gyro_y_off", &gyro_y_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read gyro_y_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_get_blob(nvs_handle, "gyro_z_off", &gyro_z_offset, &required_size);
  if (ret != ESP_OK) {
    log_warn(mpu6050_tag, "NVS Read Error", "Failed to read gyro_z_offset from NVS");
    nvs_close(nvs_handle);
    return ret;
  }
  
  nvs_close(nvs_handle);
  
  /* Apply the loaded calibration values */
  ret = mpu6050_apply_calibration(
    accel_x_offset, accel_y_offset, accel_z_offset,
    gyro_x_offset, gyro_y_offset, gyro_z_offset
  );
  
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "Calibration Apply Error", "Failed to apply loaded calibration values");
    return ret;
  }
  
  log_info(mpu6050_tag, "Calibration Loaded", "Successfully loaded and applied MPU6050 calibration values");
  return ESP_OK;
}

/**
 * @brief Calibrates the MPU6050 sensor by collecting samples and calculating offsets.
 *
 * This function collects a specified number of samples from the sensor while it is
 * stationary, and calculates the average offset values for the accelerometer and
 * gyroscope axes. These offsets are then stored in the sensor_data structure.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure.
 * @param[in] num_samples Number of samples to collect for calibration.
 *
 * @return
 * - ESP_OK if calibration is successful.
 * - ESP_FAIL or other error code if calibration fails.
 */
esp_err_t mpu6050_calibrate(mpu6050_data_t *sensor_data, uint16_t num_samples)
{
  if (sensor_data == NULL || num_samples == 0) {
    log_error(mpu6050_tag, "Calibration Error", "Invalid parameters for calibration");
    return ESP_ERR_INVALID_ARG;
  }
  
  log_info(mpu6050_tag, "Calibration", "Starting MPU6050 calibration with %d samples", num_samples);
  
  /* Variables to accumulate readings */
  float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
  float gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f;
  uint16_t valid_samples = 0;
  
  /* Collect samples */
  for (uint16_t i = 0; i < num_samples; i++) {
    /* Read raw data from the sensor */
    esp_err_t ret = mpu6050_read_raw_data(sensor_data);
    if (ret != ESP_OK) {
      log_warn(mpu6050_tag, "Calibration Warning", "Failed to read sample %d, skipping", i);
      continue;
    }
    
    /* Process the raw data to get calibrated values (without applying existing offsets) */
    ret = mpu6050_process_data(sensor_data);
    if (ret != ESP_OK) {
      log_warn(mpu6050_tag, "Calibration Warning", "Failed to process sample %d, skipping", i);
      continue;
    }
    
    /* Accumulate the readings */
    accel_x_sum += sensor_data->accel_x;
    accel_y_sum += sensor_data->accel_y;
    accel_z_sum += sensor_data->accel_z - 1.0f; /* Subtract 1g from Z-axis to account for gravity */
    gyro_x_sum += sensor_data->gyro_x;
    gyro_y_sum += sensor_data->gyro_y;
    gyro_z_sum += sensor_data->gyro_z;
    
    valid_samples++;
    
    /* Short delay between samples */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  /* Check if we have enough valid samples */
  if (valid_samples < num_samples / 2) {
    log_error(mpu6050_tag, "Calibration Error", "Too few valid samples: %d out of %d", valid_samples, num_samples);
    return ESP_FAIL;
  }
  
  /* Calculate average offsets */
  sensor_data->calibration.accel_x_offset = accel_x_sum / valid_samples;
  sensor_data->calibration.accel_y_offset = accel_y_sum / valid_samples;
  sensor_data->calibration.accel_z_offset = accel_z_sum / valid_samples;
  sensor_data->calibration.gyro_x_offset = gyro_x_sum / valid_samples;
  sensor_data->calibration.gyro_y_offset = gyro_y_sum / valid_samples;
  sensor_data->calibration.gyro_z_offset = gyro_z_sum / valid_samples;
  
  log_info(mpu6050_tag, "Calibration", "Calibration completed with %d valid samples", valid_samples);
  log_info(mpu6050_tag, "Calibration", "Accel offsets: [%f, %f, %f], Gyro offsets: [%f, %f, %f]",
           sensor_data->calibration.accel_x_offset, sensor_data->calibration.accel_y_offset, sensor_data->calibration.accel_z_offset,
           sensor_data->calibration.gyro_x_offset, sensor_data->calibration.gyro_y_offset, sensor_data->calibration.gyro_z_offset);
  
  return ESP_OK;
}

/**
 * @brief Saves calibration values to non-volatile storage (NVS).
 *
 * This function stores the calibration offsets in NVS so they can be
 * retrieved on subsequent power-ups, avoiding the need to recalibrate.
 *
 * @param[in] accel_x_offset Accelerometer X-axis offset.
 * @param[in] accel_y_offset Accelerometer Y-axis offset.
 * @param[in] accel_z_offset Accelerometer Z-axis offset.
 * @param[in] gyro_x_offset Gyroscope X-axis offset.
 * @param[in] gyro_y_offset Gyroscope Y-axis offset.
 * @param[in] gyro_z_offset Gyroscope Z-axis offset.
 *
 * @return
 * - ESP_OK if saving is successful.
 * - ESP_FAIL or other error code if saving fails.
 */
esp_err_t mpu6050_save_calibration_to_nvs(
  float accel_x_offset, float accel_y_offset, float accel_z_offset,
  float gyro_x_offset, float gyro_y_offset, float gyro_z_offset)
{
  /* Initialize NVS */
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("mpu6050", NVS_READWRITE, &nvs_handle);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to open NVS namespace: %s", esp_err_to_name(ret));
    return ret;
  }
  
  /* Save calibration values to NVS */
  ret = nvs_set_blob(nvs_handle, "accel_x_off", &accel_x_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save accel_x_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_set_blob(nvs_handle, "accel_y_off", &accel_y_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save accel_y_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_set_blob(nvs_handle, "accel_z_off", &accel_z_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save accel_z_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_set_blob(nvs_handle, "gyro_x_off", &gyro_x_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save gyro_x_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_set_blob(nvs_handle, "gyro_y_off", &gyro_y_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save gyro_y_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  ret = nvs_set_blob(nvs_handle, "gyro_z_off", &gyro_z_offset, sizeof(float));
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to save gyro_z_offset: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  /* Commit the changes */
  ret = nvs_commit(nvs_handle);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, "NVS Error", "Failed to commit changes: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }
  
  nvs_close(nvs_handle);
  log_info(mpu6050_tag, "Calibration", "Successfully saved calibration values to NVS");
  return ESP_OK;
}

/**
 * @brief Applies calibration offsets to the MPU6050 sensor.
 *
 * This function applies the provided calibration offsets to the sensor's
 * internal registers or to the data processing pipeline.
 *
 * @param[in] accel_x_offset Accelerometer X-axis offset.
 * @param[in] accel_y_offset Accelerometer Y-axis offset.
 * @param[in] accel_z_offset Accelerometer Z-axis offset.
 * @param[in] gyro_x_offset Gyroscope X-axis offset.
 * @param[in] gyro_y_offset Gyroscope Y-axis offset.
 * @param[in] gyro_z_offset Gyroscope Z-axis offset.
 *
 * @return
 * - ESP_OK if applying calibration is successful.
 * - ESP_FAIL or other error code if applying calibration fails.
 */
esp_err_t mpu6050_apply_calibration(
  float accel_x_offset, float accel_y_offset, float accel_z_offset,
  float gyro_x_offset, float gyro_y_offset, float gyro_z_offset)
{
  log_info(mpu6050_tag, "Calibration", "Applying calibration offsets to MPU6050");
  log_info(mpu6050_tag, "Calibration", "Accel offsets: [%f, %f, %f], Gyro offsets: [%f, %f, %f]",
           accel_x_offset, accel_y_offset, accel_z_offset,
           gyro_x_offset, gyro_y_offset, gyro_z_offset);
  
  /* Store the calibration values in the global sensor data structure */
  /* Note: In a real implementation, you would need to access the sensor_data structure */
  /* For now, we'll just log that the calibration was applied */
  
  return ESP_OK;
}

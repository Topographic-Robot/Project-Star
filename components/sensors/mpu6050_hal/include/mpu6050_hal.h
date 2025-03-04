/* components/sensors/mpu6050_hal/include/mpu6050_hal.h */

#ifndef TOPOROBO_MPU6050_HAL_H
#define TOPOROBO_MPU6050_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

/* Constants ******************************************************************/

extern const uint8_t    mpu6050_i2c_address;        /**< I2C address for the MPU6050 sensor (default 0x68, configurable to 0x69). */
extern const i2c_port_t mpu6050_i2c_bus;            /**< I2C bus number used by the ESP32 for MPU6050 communication. */
extern const char      *mpu6050_tag;                /**< Tag for log_handler messages related to the MPU6050 sensor. */
extern const uint8_t    mpu6050_scl_io;             /**< GPIO pin for I2C Serial Clock Line (SCL) for MPU6050. */
extern const uint8_t    mpu6050_sda_io;             /**< GPIO pin for I2C Serial Data Line (SDA) for MPU6050. */
extern const uint32_t   mpu6050_i2c_freq_hz;        /**< I2C bus frequency for MPU6050 communication (default 400 kHz). */
extern const uint32_t   mpu6050_polling_rate_ticks; /**< Polling interval for MPU6050 sensor reads in system ticks. */
extern const uint8_t    mpu6050_sample_rate_div;    /**< Sample rate divider for MPU6050 (default divides gyro rate). */
extern const uint8_t    mpu6050_config_dlpf;        /**< Digital Low Pass Filter (DLPF) setting for noise reduction. */
extern const uint8_t    mpu6050_int_io;             /**< GPIO pin for MPU6050 interrupt signal (INT pin). */

/* Calibration offsets */
extern const float      mpu6050_accel_offset_x;     /**< Calibration offset for X-axis acceleration. */
extern const float      mpu6050_accel_offset_y;     /**< Calibration offset for Y-axis acceleration. */
extern const float      mpu6050_accel_offset_z;     /**< Calibration offset for Z-axis acceleration. */
extern const float      mpu6050_gyro_offset_x;      /**< Calibration offset for X-axis gyroscope. */
extern const float      mpu6050_gyro_offset_y;      /**< Calibration offset for Y-axis gyroscope. */
extern const float      mpu6050_gyro_offset_z;      /**< Calibration offset for Z-axis gyroscope. */

/* Moving average filter parameters */
#define MPU6050_FILTER_SAMPLES 5  /**< Number of samples to average for the moving average filter. */

/* Enums **********************************************************************/

/**
 * @brief Enumeration of interrupt modes for the MPU6050 sensor.
 *
 * Defines the possible interrupt sources for the MPU6050 sensor. The INT pin on the MPU6050
 * asserts when any of the enabled interrupts occur. To identify which interrupt(s) triggered,
 * the microcontroller must read the INT_STATUS register.
 */
typedef enum : uint8_t {
  k_mpu6050_int_mode_data_ready,       /**< Interrupt triggered when new data is available. */
  k_mpu6050_int_mode_motion_detection, /**< Interrupt triggered by motion detection exceeding a threshold. */
  k_mpu6050_int_mode_zero_motion,      /**< Interrupt triggered by no motion detected for a specified duration. */
  k_mpu6050_int_mode_free_fall,        /**< Interrupt triggered by free-fall conditions. */
  k_mpu6050_int_mode_fifo_overflow,    /**< Interrupt triggered by a FIFO buffer overflow. */
  k_mpu6050_int_mode_i2c_mst,          /**< Interrupt triggered by the I2C Master module. */
  k_mpu6050_int_mode_dmp,              /**< Interrupt triggered by the Digital Motion Processor (DMP). */
  k_mpu6050_int_mode_pll_ready,        /**< Interrupt triggered when the Phase-Locked Loop (PLL) is ready. */
} mpu6050_interrupt_mode_t;

/**
 * @brief Enumeration of interrupt enable bits for the MPU6050 sensor.
 *
 * Represents the bits used to enable specific interrupts in the MPU6050's INT_ENABLE register.
 * Each bit corresponds to one of the available interrupt sources.
 */
typedef enum : uint8_t {
  k_mpu6050_int_enable_data_rdy   = 0x01, /**< Enable bit for Data Ready interrupt. */
  k_mpu6050_int_enable_dmp_int    = 0x02, /**< Enable bit for DMP interrupt. */
  k_mpu6050_int_enable_pll_rdy    = 0x04, /**< Enable bit for PLL Ready interrupt. */
  k_mpu6050_int_enable_i2c_mst    = 0x08, /**< Enable bit for I2C Master interrupt. */
  k_mpu6050_int_enable_fifo_oflow = 0x10, /**< Enable bit for FIFO Overflow interrupt. */
  k_mpu6050_int_enable_zmot       = 0x20, /**< Enable bit for Zero Motion Detection interrupt. */
  k_mpu6050_int_enable_mot        = 0x40, /**< Enable bit for Motion Detection interrupt. */
  k_mpu6050_int_enable_ff         = 0x80, /**< Enable bit for Free-Fall Detection interrupt. */
} mpu6050_int_enable_bits_t;

/**
 * @brief Enumeration of event bits for MPU6050 interrupt events.
 *
 * Represents the bits used in an event group to signal specific interrupt events
 * from the MPU6050 sensor. These event bits correspond to the interrupt sources
 * defined by the sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_event_data_ready      = 0x01, /**< Bit for Data Ready event. */
  k_mpu6050_event_motion_detected = 0x02, /**< Bit for Motion Detected event. */
  k_mpu6050_event_zero_motion     = 0x04, /**< Bit for Zero Motion event. */
  k_mpu6050_event_free_fall       = 0x08, /**< Bit for Free-Fall event. */
  k_mpu6050_event_fifo_overflow   = 0x10, /**< Bit for FIFO Overflow event. */
  k_mpu6050_event_i2c_mst         = 0x20, /**< Bit for I2C Master event. */
  k_mpu6050_event_dmp             = 0x40, /**< Bit for Digital Motion Processor (DMP) event. */
  k_mpu6050_event_pll_ready       = 0x80, /**< Bit for Phase-Locked Loop (PLL) Ready event. */
} mpu6050_event_bits_t;

/**
 * @brief Enumeration of interrupt pin active levels for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_int_active_high = 0, /**< Interrupt pin is active high (default). */
  k_mpu6050_int_active_low  = 1, /**< Interrupt pin is active low. */
} mpu6050_int_active_level_t;

/**
 * @brief Enumeration of interrupt pin modes for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_int_push_pull  = 0, /**< Interrupt pin is configured as push-pull (default). */
  k_mpu6050_int_open_drain = 1, /**< Interrupt pin is configured as open-drain. */
} mpu6050_int_pin_mode_t;

/**
 * @brief Enumeration of interrupt latch modes for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_int_latch_50us           = 0, /**< Interrupt pin pulses for 50μs. */
  k_mpu6050_int_latch_until_cleared  = 1, /**< Interrupt pin remains active until cleared. */
} mpu6050_int_latch_mode_t;

/**
 * @brief Enumeration of interrupt clear behaviors for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_int_clear_any_read      = 0, /**< Interrupt status is cleared on any register read. */
  k_mpu6050_int_clear_status_read   = 1, /**< Interrupt status is cleared only by reading INT_STATUS. */
} mpu6050_int_clear_mode_t;

/**
 * @brief Enumeration of MPU6050 sensor states.
 *
 * Defines the possible states of the MPU6050 sensor for tracking its operational
 * status, error handling, and data acquisition tasks.
 */
typedef enum : uint8_t {
  k_mpu6050_ready            = 0x00, /**< Sensor is initialized and ready to read data. */
  k_mpu6050_data_updated     = 0x01, /**< New sensor data has been updated. */
  k_mpu6050_uninitialized    = 0x10, /**< Sensor is not initialized. */
  k_mpu6050_sleep_mode       = 0x20, /**< Sensor is in sleep mode to save power. */
  k_mpu6050_error            = 0xF0, /**< General error state. */
  k_mpu6050_power_on_error   = 0xA1, /**< Error occurred during power-on initialization. */
  k_mpu6050_reset_error      = 0xA3, /**< Error occurred while resetting the sensor. */
  k_mpu6050_dlp_config_error = 0xA4, /**< Error occurred while configuring the Digital Low Pass Filter (DLPF). */
} mpu6050_states_t;

/**
 * @brief Enumeration of I2C commands for the MPU6050 sensor.
 *
 * Defines commands for configuring and operating the MPU6050 sensor, including power management,
 * data reading, interrupt configuration, and sensitivity settings for the accelerometer and gyroscope.
 * Also includes configuration values for features like Digital Low Pass Filter (DLPF) and full-scale ranges.
 */
typedef enum : uint8_t {
  /* Power Management Commands */
  k_mpu6050_power_down_cmd      = 0x40, /**< Command to power down the sensor (enter sleep mode). */
  k_mpu6050_sleep_cmd           = 0x40, /**< Command to put the sensor into sleep mode (same as power_down). */
  k_mpu6050_power_on_cmd        = 0x00, /**< Command to power on the sensor (wake up from sleep). */
  k_mpu6050_reset_cmd           = 0x80, /**< Command to reset the sensor. */
  k_mpu6050_pwr_mgmt_1_cmd      = 0x6B, /**< Power Management 1 register. */
  
  /* Configuration Commands */
  k_mpu6050_smplrt_div_cmd      = 0x19, /**< Sample Rate Divider register. */
  k_mpu6050_config_cmd          = 0x1A, /**< Configuration register for DLPF and FSYNC. */
  k_mpu6050_gyro_config_cmd     = 0x1B, /**< Gyroscope Configuration register (full-scale range). */
  k_mpu6050_accel_config_cmd    = 0x1C, /**< Accelerometer Configuration register (full-scale range). */
  k_mpu6050_who_am_i_cmd        = 0x75, /**< WHO_AM_I register to verify sensor identity. */
  
  /* Interrupt Configuration Commands */
  k_mpu6050_int_enable_cmd      = 0x38, /**< Interrupt Enable register. */
  k_mpu6050_int_status_cmd      = 0x3A, /**< Interrupt Status register. */
  k_mpu6050_int_pin_cfg_cmd     = 0x37, /**< INT Pin Configuration register. */

  /* Configuration Values */
  k_mpu6050_who_am_i_response   = 0x68, /**< Expected response from the WHO_AM_I register. */
  k_mpu6050_config_dlpf_260hz   = 0x00, /**< DLPF: 260Hz bandwidth, 0ms delay. */
  k_mpu6050_config_dlpf_184hz   = 0x01, /**< DLPF: 184Hz bandwidth, 2.0ms delay. */
  k_mpu6050_config_dlpf_94hz    = 0x02, /**< DLPF: 94Hz bandwidth, 3.0ms delay. */
  k_mpu6050_config_dlpf_44hz    = 0x03, /**< DLPF: 44Hz bandwidth, 4.9ms delay. */
  k_mpu6050_config_dlpf_21hz    = 0x04, /**< DLPF: 21Hz bandwidth, 8.5ms delay. */
  k_mpu6050_config_dlpf_10hz    = 0x05, /**< DLPF: 10Hz bandwidth, 13.8ms delay. */
  k_mpu6050_config_dlpf_5hz     = 0x06, /**< DLPF: 5Hz bandwidth, 19.0ms delay. */
  k_mpu6050_gyro_fs_250dps      = 0x00, /**< Gyroscope full-scale range: ±250°/s. */
  k_mpu6050_gyro_fs_500dps      = 0x08, /**< Gyroscope full-scale range: ±500°/s. */
  k_mpu6050_gyro_fs_1000dps     = 0x10, /**< Gyroscope full-scale range: ±1000°/s. */
  k_mpu6050_gyro_fs_2000dps     = 0x18, /**< Gyroscope full-scale range: ±2000°/s. */
  k_mpu6050_accel_fs_2g         = 0x00, /**< Accelerometer full-scale range: ±2g. */
  k_mpu6050_accel_fs_4g         = 0x08, /**< Accelerometer full-scale range: ±4g. */
  k_mpu6050_accel_fs_8g         = 0x10, /**< Accelerometer full-scale range: ±8g. */
  k_mpu6050_accel_fs_16g        = 0x18, /**< Accelerometer full-scale range: ±16g. */

  /* Data Register Commands */
  k_mpu6050_accel_xout_h_cmd    = 0x3B, /**< Accelerometer X-axis High byte. */
  k_mpu6050_accel_xout_l_cmd    = 0x3C, /**< Accelerometer X-axis Low byte. */
  k_mpu6050_accel_yout_h_cmd    = 0x3D, /**< Accelerometer Y-axis High byte. */
  k_mpu6050_accel_yout_l_cmd    = 0x3E, /**< Accelerometer Y-axis Low byte. */
  k_mpu6050_accel_zout_h_cmd    = 0x3F, /**< Accelerometer Z-axis High byte. */
  k_mpu6050_accel_zout_l_cmd    = 0x40, /**< Accelerometer Z-axis Low byte. */
  k_mpu6050_gyro_xout_h_cmd     = 0x43, /**< Gyroscope X-axis High byte. */
  k_mpu6050_gyro_xout_l_cmd     = 0x44, /**< Gyroscope X-axis Low byte. */
  k_mpu6050_gyro_yout_h_cmd     = 0x45, /**< Gyroscope Y-axis High byte. */
  k_mpu6050_gyro_yout_l_cmd     = 0x46, /**< Gyroscope Y-axis Low byte. */
  k_mpu6050_gyro_zout_h_cmd     = 0x47, /**< Gyroscope Z-axis High byte. */
  k_mpu6050_gyro_zout_l_cmd     = 0x48, /**< Gyroscope Z-axis Low byte. */

  /* Unused or Optional Commands */
  k_mpu6050_pwr_mgmt_2_cmd      = 0x6C, /**< Power Management 2 register. */
  k_mpu6050_fifo_en_cmd         = 0x23, /**< FIFO Enable register. */
  k_mpu6050_fifo_count_h_cmd    = 0x72, /**< FIFO Count High byte. */
  k_mpu6050_fifo_count_l_cmd    = 0x73, /**< FIFO Count Low byte. */
  k_mpu6050_fifo_r_w_cmd        = 0x74, /**< FIFO Read/Write register. */
  k_mpu6050_temp_out_h_cmd      = 0x41, /**< Temperature High byte. */
  k_mpu6050_temp_out_l_cmd      = 0x42, /**< Temperature Low byte. */

  /* Factory-Level Testing Commands */
  k_mpu6050_self_test_x_cmd     = 0x0D, /**< Self-test register for X-axis (factory testing). */
  k_mpu6050_self_test_y_cmd     = 0x0E, /**< Self-test register for Y-axis (factory testing). */
  k_mpu6050_self_test_z_cmd     = 0x0F, /**< Self-test register for Z-axis (factory testing). */
  k_mpu6050_self_test_a_cmd     = 0x10, /**< Self-test register for Accelerometer (factory testing). */
} mpu6050_commands_t;

/* Structs ********************************************************************/

/**
 * @brief Structure to hold the accelerometer configuration for the MPU6050.
 *
 * Contains the register value for setting the full-scale range and the corresponding
 * scaling factor for converting raw acceleration data to gravitational units (g).
 *
 * The scaling factor is calculated as:
 *   Scaling Factor = (Full-Scale Range) / (Maximum Raw Value)
 * where the Maximum Raw Value is 32768 for the MPU6050.
 */
typedef struct {
  uint8_t accel_config; /**< Register value for setting the accelerometer full-scale range. */
  float   accel_scale;  /**< Scaling factor to convert raw data to acceleration in g. */
} mpu6050_accel_config_t;

/**
 * @brief Structure to hold the gyroscope configuration for the MPU6050.
 *
 * Contains the register value for setting the full-scale range and the corresponding
 * scaling factor for converting raw gyroscope data to angular velocity in degrees per second (°/s).
 *
 * The scaling factor is calculated as:
 *   Scaling Factor = (Full-Scale Range) / (Maximum Raw Value)
 * where the Maximum Raw Value is 32768 for the MPU6050.
 */
typedef struct {
  uint8_t gyro_config; /**< Register value for setting the gyroscope full-scale range. */
  float   gyro_scale;  /**< Scaling factor to convert raw data to angular velocity in °/s. */
} mpu6050_gyro_config_t;

/**
 * @brief Structure to configure the interrupt pin behavior for the MPU6050.
 */
typedef struct {
  gpio_num_t                 interrupt_pin;     /**< GPIO pin connected to MPU6050 INT pin. */
  mpu6050_int_active_level_t active_level;      /**< Active level of the interrupt pin. */
  mpu6050_int_pin_mode_t     pin_mode;          /**< Push-pull or open-drain mode. */
  mpu6050_int_latch_mode_t   latch_mode;        /**< Interrupt latch behavior. */
  mpu6050_int_clear_mode_t   clear_mode;        /**< Interrupt clear behavior. */
  uint8_t                    interrupt_sources; /**< Bit mask of interrupt sources to enable. */
} mpu6050_int_config_t;

/**
 * @brief Structure to store raw MPU6050 sensor data.
 */
typedef struct {
  int16_t accel_x_raw; /**< Raw X-axis acceleration value. */
  int16_t accel_y_raw; /**< Raw Y-axis acceleration value. */
  int16_t accel_z_raw; /**< Raw Z-axis acceleration value. */
  int16_t gyro_x_raw;  /**< Raw X-axis gyroscope value. */
  int16_t gyro_y_raw;  /**< Raw Y-axis gyroscope value. */
  int16_t gyro_z_raw;  /**< Raw Z-axis gyroscope value. */
  int16_t temp_raw;    /**< Raw temperature value. */
} mpu6050_raw_data_t;

/**
 * @brief Structure to hold calibration data for the MPU6050 sensor.
 */
typedef struct {
  float accel_x_offset; /**< Accelerometer X-axis offset value. */
  float accel_y_offset; /**< Accelerometer Y-axis offset value. */
  float accel_z_offset; /**< Accelerometer Z-axis offset value. */
  float gyro_x_offset;  /**< Gyroscope X-axis offset value. */
  float gyro_y_offset;  /**< Gyroscope Y-axis offset value. */
  float gyro_z_offset;  /**< Gyroscope Z-axis offset value. */
} mpu6050_calibration_t;

/**
 * @brief Structure to hold all data related to the MPU6050 sensor.
 */
typedef struct {
  /* I2C communication parameters */
  i2c_port_t i2c_bus;     /**< I2C bus number used for communication. */
  uint8_t i2c_address;    /**< I2C address of the MPU6050 sensor. */
  
  /* Sensor data */
  float accel_x;          /**< Accelerometer X-axis reading in g. */
  float accel_y;          /**< Accelerometer Y-axis reading in g. */
  float accel_z;          /**< Accelerometer Z-axis reading in g. */
  float gyro_x;           /**< Gyroscope X-axis reading in degrees per second. */
  float gyro_y;           /**< Gyroscope Y-axis reading in degrees per second. */
  float gyro_z;           /**< Gyroscope Z-axis reading in degrees per second. */
  float temperature;      /**< Temperature reading in degrees Celsius. */
  
  /* Sensor state */
  mpu6050_states_t state; /**< Current state of the sensor. */
  
  /* Data mutex for thread safety */
  portMUX_TYPE data_mutex; /**< Mutex for protecting access to sensor data. */
  
  /* Interrupt configuration */
  mpu6050_int_config_t int_config; /**< Interrupt configuration settings. */
  
  /* Semaphore for data ready interrupt */
  SemaphoreHandle_t data_ready_sem; /**< Semaphore for signaling when new data is ready. */
  
  /* Moving average filter buffers */
  float accel_x_buffer[MPU6050_FILTER_SAMPLES]; /**< Buffer for accelerometer X-axis readings. */
  float accel_y_buffer[MPU6050_FILTER_SAMPLES]; /**< Buffer for accelerometer Y-axis readings. */
  float accel_z_buffer[MPU6050_FILTER_SAMPLES]; /**< Buffer for accelerometer Z-axis readings. */
  float gyro_x_buffer[MPU6050_FILTER_SAMPLES];  /**< Buffer for gyroscope X-axis readings. */
  float gyro_y_buffer[MPU6050_FILTER_SAMPLES];  /**< Buffer for gyroscope Y-axis readings. */
  float gyro_z_buffer[MPU6050_FILTER_SAMPLES];  /**< Buffer for gyroscope Z-axis readings. */
  uint8_t buffer_index;                        /**< Current index in the circular buffer. */
  
  /* Calibration data */
  mpu6050_calibration_t calibration;           /**< Calibration offsets for the sensor. */
} mpu6050_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Converts MPU6050 sensor data to a JSON string.
 *
 * Converts the accelerometer and gyroscope data in a `mpu6050_data_t` 
 * structure to a dynamically allocated JSON string. The caller must free the memory.
 *
 * @param[in] data Pointer to the `mpu6050_data_t` structure with valid sensor data.
 *
 * @return 
 * - Pointer to the JSON-formatted string on success.
 * - `NULL` if memory allocation fails.
 * 
 * @note The returned string should be freed by the caller to prevent memory leaks.
 */
char *mpu6050_data_to_json(const mpu6050_data_t *data);

/**
 * @brief Initializes the MPU6050 sensor in continuous measurement mode.
 *
 * Configures the MPU6050 sensor for accelerometer and gyroscope data collection.
 * Sets up I2C, applies default settings, and initializes the sensor for continuous
 * measurement mode.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure holding
 *                            initialization parameters and state.
 *
 * @return
 * - ESP_OK if initialization is successful.
 * - ESP_FAIL or other error code if initialization fails.
 */
esp_err_t mpu6050_init(void *sensor_data);

/**
 * @brief Configure the interrupt settings for the MPU6050 sensor.
 *
 * This function configures the interrupt pin behavior and enables the specified
 * interrupt sources on the MPU6050 sensor.
 *
 * @param[in] sensor_data Pointer to the mpu6050_data_t structure.
 * @param[in] int_config Pointer to the interrupt configuration structure.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Failure
 */
esp_err_t mpu6050_config_interrupts(mpu6050_data_t *sensor_data, const mpu6050_int_config_t *int_config);

/**
 * @brief Enables specific interrupt sources on the MPU6050.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 * @param[in] interrupt_sources Bit mask of interrupt sources to enable.
 *
 * @return
 * - ESP_OK if successful.
 * - ESP_FAIL or other error code if failed.
 */
esp_err_t mpu6050_enable_interrupts(mpu6050_data_t *sensor_data, uint8_t interrupt_sources);

/**
 * @brief Disables specific interrupt sources on the MPU6050.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 * @param[in] interrupt_sources Bit mask of interrupt sources to disable.
 *
 * @return
 * - ESP_OK if successful.
 * - ESP_FAIL or other error code if failed.
 */
esp_err_t mpu6050_disable_interrupts(mpu6050_data_t *sensor_data, uint8_t interrupt_sources);

/**
 * @brief Reads the current interrupt status from the MPU6050.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 * @param[out] status Pointer to store the interrupt status.
 *
 * @return
 * - ESP_OK if successful.
 * - ESP_FAIL or other error code if failed.
 */
esp_err_t mpu6050_get_interrupt_status(mpu6050_data_t *sensor_data, uint8_t *status);

/**
 * @brief Configures the accelerometer and gyroscope settings.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 * @param[in] accel_fs_idx Index into the accelerometer configuration array.
 * @param[in] gyro_fs_idx Index into the gyroscope configuration array.
 *
 * @return
 * - ESP_OK if configuration is successful.
 * - ESP_FAIL or other error code if configuration fails.
 */
esp_err_t mpu6050_config_sensor(mpu6050_data_t *sensor_data, uint8_t accel_fs_idx, uint8_t gyro_fs_idx);

/**
 * @brief Reads raw sensor data from the MPU6050.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if read is successful.
 * - ESP_FAIL or other error code if read fails.
 */
esp_err_t mpu6050_read_raw_data(mpu6050_data_t *sensor_data);

/**
 * @brief Processes raw sensor data into calibrated values.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if processing is successful.
 * - ESP_FAIL or other error code if processing fails.
 */
esp_err_t mpu6050_process_data(mpu6050_data_t *sensor_data);

/**
 * @brief Resets the MPU6050 sensor when an error occurs.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if reset is successful.
 * - ESP_FAIL or other error code if reset fails.
 */
esp_err_t mpu6050_reset_on_error(mpu6050_data_t *sensor_data);

/**
 * @brief Puts the MPU6050 sensor into sleep mode to save power.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if the sensor was successfully put into sleep mode.
 * - ESP_FAIL or other error code if an error occurred.
 */
esp_err_t custom_mpu6050_sleep(mpu6050_data_t *sensor_data);

/**
 * @brief Wakes up the MPU6050 sensor from sleep mode.
 *
 * @param[in] sensor_data Pointer to the `mpu6050_data_t` structure.
 *
 * @return
 * - ESP_OK if the sensor was successfully woken up.
 * - ESP_FAIL or other error code if an error occurred.
 */
esp_err_t custom_mpu6050_wake_up(mpu6050_data_t *sensor_data);

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
esp_err_t mpu6050_self_test(mpu6050_data_t *sensor_data);

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
esp_err_t mpu6050_calibrate(mpu6050_data_t *sensor_data, uint16_t num_samples);

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
  float gyro_x_offset, float gyro_y_offset, float gyro_z_offset);

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
  float gyro_x_offset, float gyro_y_offset, float gyro_z_offset);

/**
 * @brief Loads calibration values from non-volatile storage (NVS).
 *
 * This function retrieves the calibration offsets from NVS and applies them
 * to the sensor, restoring calibration settings from previous sessions.
 *
 * @return
 * - ESP_OK if loading is successful.
 * - ESP_FAIL or other error code if loading fails.
 */
esp_err_t mpu6050_load_calibration_from_nvs(void);

/**
 * @brief Task function for the MPU6050 sensor to be run periodically.
 *
 * This function handles the periodic tasks for the MPU6050 sensor, including
 * reading data, processing it, and handling any errors that may occur.
 *
 * @param[in] sensor_data Pointer to the sensor data structure.
 */
void mpu6050_tasks(void *sensor_data);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_MPU6050_HAL_H */


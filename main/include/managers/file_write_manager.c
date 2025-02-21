/* main/include/managers/file_write_manager.c */

/* TODO: Test this */

#include "file_write_manager.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "sd_card_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const char    *file_manager_tag   = "FILE_MANAGER";
const uint32_t max_pending_writes = 10; /**< Maximum queued write operations */

/* Globals (Static) ***********************************************************/

static QueueHandle_t        s_file_write_queue       = NULL; /**< Queue for handling file write requests */
static TaskHandle_t         s_file_write_task_handle = NULL;
static file_writer_config_t s_file_writer_config     = { 3, 4096, false };

/* Private Functions **********************************************************/

/**
 * @brief Formats the current date and time as a string.
 *
 * Retrieves the current system time and formats it as `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[out] buffer Buffer to store the formatted timestamp.
 * @param[in]  buffer_len Length of the buffer.
 *
 * @note 
 * - The system time must be properly initialized before calling this function.
 */
static void priv_get_timestamp(char *buffer, size_t buffer_len)
{
  time_t    now = time(NULL);
  struct tm timeinfo;

  localtime_r(&now, &timeinfo);
  strftime(buffer, buffer_len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

/**
 * @brief Task to handle queued file write requests.
 *
 * Processes file write requests from a queue and writes data to the specified 
 * files. The task runs continuously, handling requests asynchronously.
 *
 * @param[in] param Pointer to task-specific parameters (unused)
 *
 * @note 
 * - This function is intended to run as a FreeRTOS task.
 * - Each write request includes a timestamp and data to be written.
 */
static void priv_file_write_task(void *param)
{
  ESP_LOGI(file_manager_tag, "- File writer task started - processing queue");
  file_write_request_t request;

  while (1) {
    if (xQueueReceive(s_file_write_queue, &request, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(file_manager_tag, "- Processing write request - target: %s", request.file_path);
      
      FILE *file = fopen(request.file_path, "a");
      if (file == NULL) {
        ESP_LOGE(file_manager_tag, "- Write operation failed - could not open file: %s", request.file_path);
        continue;
      }

      size_t bytes_written = fwrite(request.data, 1, strlen(request.data), file);
      fclose(file);

      if (bytes_written != strlen(request.data)) {
        ESP_LOGE(file_manager_tag, "- Write operation incomplete - wrote %zu/%zu bytes to %s", 
                 bytes_written, strlen(request.data), request.file_path);
      } else {
        ESP_LOGI(file_manager_tag, "- Write operation successful - %zu bytes written to %s", 
                 bytes_written, request.file_path);
      }
    }
  }
}

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(void)
{
  if (!s_file_writer_config.enabled) {
    ESP_LOGI(file_manager_tag, "- File writer disabled - skipping initialization");
    return ESP_OK;
  }

  ESP_LOGI(file_manager_tag, "- Starting file writer initialization");

  s_file_write_queue = xQueueCreate(max_pending_writes, sizeof(file_write_request_t));
  if (s_file_write_queue == NULL) {
    ESP_LOGE(file_manager_tag, "- Queue creation failed - insufficient memory");
    return ESP_FAIL;
  }
  ESP_LOGI(file_manager_tag, "- Write queue created - capacity: %lu requests", max_pending_writes);

  BaseType_t task_created = xTaskCreate(priv_file_write_task,
                                        file_manager_tag,
                                        s_file_writer_config.stack_depth,
                                        NULL,
                                        s_file_writer_config.priority,
                                        &s_file_write_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(file_manager_tag, "- Task creation failed - insufficient resources");
    return ESP_FAIL;
  }
  ESP_LOGI(file_manager_tag, "- Write task created - priority: %u", s_file_writer_config.priority);

  if (sd_card_init() != ESP_OK) {
    ESP_LOGE(file_manager_tag, "- SD card initialization failed - storage unavailable");
    return ESP_FAIL;
  }
  ESP_LOGI(file_manager_tag, "- SD card initialized - storage system ready");

  ESP_LOGI(file_manager_tag, "- File writer initialization complete - system operational");
  return ESP_OK;
}

esp_err_t file_write_enqueue(const char *file_path, const char *data)
{
  if (!s_file_writer_config.enabled) {
    ESP_LOGW(file_manager_tag, "- Write request rejected - file writer is disabled");
    return ESP_FAIL;
  }

  if (file_path == NULL || data == NULL) {
    ESP_LOGE(file_manager_tag, "- Write request rejected - invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  if (s_file_write_queue == NULL) {
    ESP_LOGE(file_manager_tag, "- Write request rejected - queue not initialized");
    return ESP_FAIL;
  }

  file_write_request_t request;
  char timestamp[32] = { '\0' };
  
  priv_get_timestamp(timestamp, sizeof(timestamp));
  snprintf(request.file_path, MAX_FILE_PATH_LENGTH, "%s/%s",   sd_card_mount_path, file_path);
  snprintf(request.data,      MAX_DATA_LENGTH,      "%s %s\n", timestamp,          data);

  ESP_LOGI(file_manager_tag, "- Enqueueing write request - target: %s", file_path);
  if (xQueueSend(s_file_write_queue, &request, 0) != pdTRUE) {
    ESP_LOGE(file_manager_tag, "- Write request rejected - queue full");
    return ESP_FAIL;
  }

  ESP_LOGI(file_manager_tag, "- Write request accepted - queued for processing");
  return ESP_OK;
}


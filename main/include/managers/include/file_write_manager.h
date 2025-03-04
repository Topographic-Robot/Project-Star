/* main/include/managers/include/file_write_manager.h */

#ifndef TOPOROBO_FILE_WRITE_MANAGER_H
#define TOPOROBO_FILE_WRITE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* Constants ******************************************************************/

extern const char    *file_manager_tag;   /**< Logging tag for log_handler messages related to the file write manager. */
extern const uint32_t max_pending_writes; /**< Maximum number of queued file write requests to prevent overflow. */

/* Macros *********************************************************************/

#define MAX_FILE_PATH_LENGTH  (64)  /**< Maximum file path length, including the null terminator. */
#define MAX_DATA_LENGTH       (256) /**< Maximum data length per write request, including the null terminator. */
#define TIMESTAMP_BUFFER_SIZE (64)  /**< Size of buffer for timestamp strings. */

/* Structs ********************************************************************/

/**
 * @brief Configuration structure for the file write manager task.
 *
 * Contains settings for the file write manager task including its name,
 * priority, stack size, and enablement flag.
 */
typedef struct {
  UBaseType_t priority;    /**< Priority of the file writer task for scheduling purposes. */
  uint32_t    stack_depth; /**< Stack depth allocated for the file writer task, in words. */
  bool        enabled;     /**< Flag indicating if the file writer is enabled (true) or disabled (false). */
} file_writer_config_t;

/**
 * @brief Represents a request to write data to a file.
 *
 * Contains the file path, data, and data length for a file write operation.
 * The is_binary flag indicates whether the data should be treated as binary or text.
 * 
 * @note
 * - The `file_path` must be null-terminated and have a length defined by 
 *   the `max_file_path_length` macro to prevent memory overflow.
 * - For text data (is_binary = false), the data field contains the null-terminated string.
 * - For binary data (is_binary = true), the data field contains a pointer to the binary data.
 */
typedef struct {
  char     file_path[MAX_FILE_PATH_LENGTH]; /**< Path to the target file. Must be null-terminated and within the length limit. */
  void    *data;                            /**< Pointer to the data to write. For text, this is a null-terminated string. */
  uint32_t data_length;                     /**< Length of the data in bytes. For text, this can be 0 (will use strlen). */
  bool     is_binary;                       /**< Flag indicating if this is a binary write request (true) or text (false). */
} file_write_request_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the file write manager.
 *
 * Sets up a FreeRTOS queue for managing asynchronous file write requests 
 * and starts a background task to process them. Files are always opened 
 * in append mode, creating them if they do not exist. Each line of data 
 * written to a file will include a timestamp at the start in the format 
 * `YYYY-MM-DD HH:MM:SS`.
 *
 * @return
 * - ESP_OK   if the initialization is successful.
 * - ESP_FAIL if the queue creation fails.
 *
 * @note This function must be called before attempting to enqueue file 
 *       write requests.
 */
esp_err_t file_write_manager_init(void);

/**
 * @brief Enqueues a file write request.
 *
 * Adds a file write request to the queue for asynchronous processing. 
 * The data will be written to the specified file in the background by 
 * the file write task. If the file does not exist, it will be created 
 * automatically. All writes append data to the file. Each line written 
 * includes a timestamp at the beginning in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in] file_path Path to the file (e.g., "/sdcard/sensor1.txt"). 
 *                      This must be a valid file path accessible by 
 *                      the system.
 * @param[in] data      Null-terminated string to write to the file. The 
 *                      string should not exceed the maximum size allowed 
 *                      by the queue.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers).
 * - ESP_FAIL            if the queue is full.
 *
 * @note Ensure `file_write_manager_init` has been called before invoking 
 *       this function. The function does not block but returns immediately 
 *       after enqueueing the request.
 */
esp_err_t file_write_enqueue(const char *file_path, const char *data);

/**
 * @brief Enqueues a binary file write request.
 *
 * Adds a binary file write request to the queue for asynchronous processing.
 * The binary data will be written to the specified file in the background by
 * the file write task. If the file does not exist, it will be created
 * automatically. All writes append data to the file.
 *
 * @param[in] file_path   Path to the file (e.g., "/sdcard/logs/log.gz").
 *                        This must be a valid file path accessible by
 *                        the system.
 * @param[in] data        Pointer to the binary data to write.
 * @param[in] data_length Length of the binary data in bytes.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers).
 * - ESP_FAIL            if the queue is full.
 *
 * @note Ensure `file_write_manager_init` has been called before invoking
 *       this function. The function does not block but returns immediately
 *       after enqueueing the request. The data is copied to an internal buffer,
 *       so the caller can free the original data after this function returns.
 */
esp_err_t file_write_binary_enqueue(const char *file_path, const void *data, uint32_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_FILE_WRITE_MANAGER_H */


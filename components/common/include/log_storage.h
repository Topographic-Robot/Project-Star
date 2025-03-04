/* components/common/include/log_storage.h */

#ifndef TOPOROBO_LOG_STORAGE_H
#define TOPOROBO_LOG_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include "sd_card_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "zlib.h"

/* Constants ******************************************************************/

/* External configuration constants */
extern const char *log_storage_tag;          /* Tag for logging */
extern const char *log_base_dir;             /* Base directory for logs */
extern const int   log_max_file_size;        /* Maximum log file size in bytes (1MB) */
extern const int   log_max_files;            /* Maximum number of log files to keep */
extern const int   date_string_buffer_size;  /* Size of buffer for date strings */
extern const int   log_compression_enabled;  /* Enable/disable compression (1=enabled, 0=disabled) */
extern const int   log_compression_level;    /* Compression level (0-9, or Z_DEFAULT_COMPRESSION) */
extern const int   log_compression_buffer;   /* Size of compression buffer */
extern const char *log_compressed_extension; /* Extension for compressed log files */

/* Macros *********************************************************************/

/* Buffer and message configuration */
#define LOG_BUFFER_SIZE                 10   /* Size of the log buffer for temporary storage */
#define LOG_STORAGE_MAX_MESSAGE_LENGTH  256  /* Maximum length of log messages */

/* Format strings for date and time */
#define DATE_FORMAT                     "%04d-%02d-%02d"
#define TIME_FORMAT                     "%02d:%02d:%02d"
#define TIMESTAMP_FORMAT                "[" DATE_FORMAT " " TIME_FORMAT ".%03llu]"
#define LOG_ENTRY_FORMAT                "%s [%s] %s"
#define LOG_FILENAME_FORMAT             "log_%04d%02d%02d_%02d%02d%02d%s"

/* Helper macros for formatting date and time */
#define FORMAT_DATE_ARGS(tm_ptr)        ((tm_ptr)->tm_year + 1900), ((tm_ptr)->tm_mon + 1), ((tm_ptr)->tm_mday)
#define FORMAT_TIME_ARGS(tm_ptr)        ((tm_ptr)->tm_hour), ((tm_ptr)->tm_min), ((tm_ptr)->tm_sec)

/* Structs ********************************************************************/

typedef struct {
  char            buffer[LOG_STORAGE_MAX_MESSAGE_LENGTH];
  esp_log_level_t level;
  uint64_t        timestamp;
} log_entry_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log storage system
 * 
 * Sets up the log storage system for saving logs to the SD card.
 * This includes creating necessary directories and initializing
 * internal buffers and synchronization primitives.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_init(void);

/**
 * @brief Updates the SD card availability status
 * 
 * This function should be called when the SD card is inserted or removed
 * to update the internal state of the log storage system. When the SD card
 * becomes available, any buffered logs will be flushed to the card.
 * 
 * @param available true if the SD card is available, false otherwise
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_set_sd_available(bool available);

/**
 * @brief Writes a log message to storage
 * 
 * Stores the log message in an internal buffer and flushes to the SD card
 * when appropriate. If the SD card is not available, logs are kept in the
 * buffer until the card becomes available or the buffer is full.
 * 
 * @param level The log level (ERROR, WARN, INFO, DEBUG, VERBOSE)
 * @param message The log message to store
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_write(esp_log_level_t level, const char *message);

/**
 * @brief Flushes any buffered logs to the SD card
 * 
 * Forces an immediate flush of all buffered logs to the SD card.
 * This is useful when shutting down the system or when immediate
 * persistence is required.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_flush(void);

/**
 * @brief Enables or disables log compression
 * 
 * Controls whether log files are compressed when written to the SD card.
 * When enabled, log files are compressed using zlib with the .gz extension.
 * 
 * @param enabled true to enable compression, false to disable
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_set_compression(bool enabled);

/**
 * @brief Gets the current compression status
 * 
 * @return true if compression is enabled, false otherwise
 */
bool log_storage_is_compression_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_LOG_STORAGE_H */ 
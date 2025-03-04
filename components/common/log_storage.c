/* components/common/log_storage.c */

#include "log_storage.h"
#include "log_handler.h"
#include "file_write_manager.h"
#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

/* Constants ******************************************************************/

const char *log_storage_tag = "LOG_STORAGE";
const char *log_base_dir    = "logs";
const int log_max_file_size = 1024 * 1024; /* Maximum log file size in bytes (1MB) */
const int log_max_files = 10;              /* Maximum number of log files to keep */
const int date_string_buffer_size = 32;    /* Size of buffer for date strings */
const int log_compression_enabled = 1;     /* Enable/disable compression (1=enabled, 0=disabled) */
const int log_compression_level = Z_DEFAULT_COMPRESSION; /* Compression level (0-9, or Z_DEFAULT_COMPRESSION) */
const int log_compression_buffer = 4096;   /* Size of compression buffer */
const char *log_compressed_extension = ".gz"; /* Extension for compressed log files */

/* Globals (Static) ***********************************************************/

static log_entry_t       s_log_buffer[LOG_BUFFER_SIZE]            = {0};                     /* Buffer to store logs when SD card is not available */
static uint32_t          s_log_buffer_index                       = 0;                       /* Current index in the buffer */
static bool              s_sd_card_available                      = false;                   /* Flag to track SD card availability */
static bool              s_log_storage_initialized                = false;                   /* Flag to track initialization status */
static char              s_current_log_file[MAX_FILE_PATH_LENGTH] = {0};                     /* Current log file path */
static SemaphoreHandle_t s_log_mutex                              = NULL;                    /* Mutex for thread-safe access */
static bool              s_compression_enabled                    = true; /* Compression state */

/* Private Functions **********************************************************/

/**
 * @brief Converts a log level to its string representation
 * 
 * @param[in] level The log level
 * @return Const string representing the log level
 */
static const char *priv_log_level_to_string(esp_log_level_t level)
{
  switch (level) {
    case ESP_LOG_ERROR:   return "ERROR";
    case ESP_LOG_WARN:    return "WARN";
    case ESP_LOG_INFO:    return "INFO";
    case ESP_LOG_DEBUG:   return "DEBUG";
    case ESP_LOG_VERBOSE: return "VERBOSE";
    default:              return "UNKNOWN";
  }
}

/**
 * @brief Creates the log directory structure if it doesn't exist
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_create_log_directories(void)
{
  ESP_LOGI(log_storage_tag, "Creating log directories");
  
  /* Check if base directory exists */
  struct stat st;
  if (stat(log_base_dir, &st) != 0) {
    /* Directory doesn't exist, create it */
    if (mkdir(log_base_dir, 0755) != 0) {
      ESP_LOGE(log_storage_tag, "Failed to create log directory: %s (errno: %d)", log_base_dir, errno);
      return ESP_FAIL;
    }
  }
  
  /* Create date-based subdirectory */
  struct tm timeinfo;
  time_t now = time(NULL);
  localtime_r(&now, &timeinfo);
  
  char date_str[date_string_buffer_size];
  snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", 
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  
  char subdir[MAX_FILE_PATH_LENGTH];
  snprintf(subdir, sizeof(subdir), "%s/%s", log_base_dir, date_str);
  
  if (stat(subdir, &st) != 0) {
    /* Subdirectory doesn't exist, create it */
    if (mkdir(subdir, 0755) != 0) {
      ESP_LOGE(log_storage_tag, "Failed to create log subdirectory: %s (errno: %d)", subdir, errno);
      return ESP_FAIL;
    }
  }
  
  return ESP_OK;
}

/**
 * @brief Generates a log file path based on the current time
 * 
 * @param[out] file_path Buffer to store the generated file path
 * @param[in] file_path_len Length of the file_path buffer
 */
static void priv_generate_log_file_path(char *file_path, size_t file_path_len)
{
  struct tm timeinfo;
  time_t now = time(NULL);
  localtime_r(&now, &timeinfo);
  
  char date_str[date_string_buffer_size];
  snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", 
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  
  const char *extension = s_compression_enabled ? log_compressed_extension : ".txt";
  
  snprintf(file_path, file_path_len, "%s/%s/" LOG_FILENAME_FORMAT,
           log_base_dir, date_str,
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
           extension);
}

/**
 * @brief Checks if log rotation is needed
 * 
 * @return true if rotation is needed, false otherwise
 */
static bool priv_check_log_rotation(void)
{
  /* If no current log file, no need to rotate */
  if (strlen(s_current_log_file) == 0) {
    return false;
  }
  
  /* Check file size */
  struct stat st;
  if (stat(s_current_log_file, &st) != 0) {
    /* File doesn't exist, no need to rotate */
    return false;
  }
  
  /* Check if file size exceeds the maximum */
  if (st.st_size >= log_max_file_size) {
    return true;
  }
  
  /* Check if date has changed */
  struct tm timeinfo;
  time_t now = time(NULL);
  localtime_r(&now, &timeinfo);
  
  char date_str[date_string_buffer_size]; /* Increased from 20 to ensure enough space */
  snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", 
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  
  /* Extract date from current log file path */
  char *date_start = strstr(s_current_log_file, log_base_dir);
  if (date_start == NULL) {
    return false;
  }
  
  /* If date has changed, rotate */
  if (strstr(s_current_log_file, date_str) == NULL) {
    return true;
  }
  
  return false;
}

/**
 * @brief Rotates the log file if needed
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_rotate_log_file(void)
{
  if (!priv_check_log_rotation()) {
    return ESP_OK; /* No rotation needed */
  }
  
  /* Create directories if needed */
  if (priv_create_log_directories() != ESP_OK) {
    return ESP_FAIL;
  }
  
  /* Generate new log file path */
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));
  log_info(log_storage_tag, "Log Rotation", "Rotating to new log file: %s", s_current_log_file);
  
  return ESP_OK;
}

/**
 * @brief Compresses data using zlib
 * 
 * @param[in] input Input data to compress
 * @param[in] input_len Length of input data
 * @param[out] output Buffer to store compressed data
 * @param[in,out] output_len Size of output buffer on input, size of compressed data on output
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_compress_data(const char *input, size_t input_len, 
                                   char *output, size_t *output_len)
{
  if (input == NULL || output == NULL || output_len == NULL) {
    return ESP_FAIL;
  }
  
  z_stream stream;
  memset(&stream, 0, sizeof(stream));
  
  /* Initialize zlib for gzip compression */
  int ret = deflateInit2(&stream, log_compression_level, Z_DEFLATED,
                         15 + 16, /* 15 for max window size, +16 for gzip header */
                         8, Z_DEFAULT_STRATEGY);
  if (ret != Z_OK) {
    ESP_LOGE(log_storage_tag, "Failed to initialize zlib: %d", ret);
    return ESP_FAIL;
  }
  
  /* Set up input and output buffers */
  stream.next_in = (Bytef *)input;
  stream.avail_in = input_len;
  stream.next_out = (Bytef *)output;
  stream.avail_out = *output_len;
  
  /* Compress data */
  ret = deflate(&stream, Z_FINISH);
  
  /* Clean up */
  deflateEnd(&stream);
  
  if (ret != Z_STREAM_END) {
    ESP_LOGE(log_storage_tag, "Failed to compress data: %d", ret);
    return ESP_FAIL;
  }
  
  /* Update output length */
  *output_len = stream.total_out;
  
  return ESP_OK;
}

/**
 * @brief Writes log data to a file
 * 
 * @param[in] file_path Path to the log file
 * @param[in] data Data to write
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_log_data(const char *file_path, const char *data)
{
  if (file_path == NULL || data == NULL) {
    return ESP_FAIL;
  }
  
  /* Check if compression is enabled */
  if (s_compression_enabled) {
    /* Compress data before writing */
    char *compress_buffer = malloc(log_compression_buffer);
    if (compress_buffer == NULL) {
      ESP_LOGE(log_storage_tag, "Failed to allocate compression buffer");
      return ESP_FAIL;
    }
    
    size_t data_len = strlen(data);
    size_t output_len = log_compression_buffer;
    
    esp_err_t ret = priv_compress_data(data, data_len, compress_buffer, &output_len);
    if (ret != ESP_OK) {
      free(compress_buffer);
      return ret;
    }
    
    /* Write compressed data to file */
    ret = file_write_binary_enqueue(file_path, compress_buffer, output_len);
    free(compress_buffer);
    return ret;
  } else {
    /* Write uncompressed data to file */
    return file_write_enqueue(file_path, data);
  }
}

/**
 * @brief Flushes the buffered logs to the SD card
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_flush_log_buffer(void)
{
  if (s_log_buffer_index == 0) {
    return ESP_OK; /* Nothing to flush */
  }
  
  if (!s_sd_card_available) {
    log_warn(log_storage_tag, "Flush Skip", "SD card not available, keeping %lu logs in buffer", 
             s_log_buffer_index);
    return ESP_FAIL;
  }
  
  /* Ensure we have a valid log file */
  if (priv_rotate_log_file() != ESP_OK) {
    return ESP_FAIL;
  }
  
  /* If compression is enabled, we'll collect all logs into a single buffer first */
  char *all_logs = NULL;
  size_t total_size = 0;
  
  if (s_compression_enabled) {
    /* Calculate total size needed */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char *level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
      /* Format: [TIMESTAMP] [LEVEL] MESSAGE */
      time_t log_time = s_log_buffer[i].timestamp / 1000000; /* Convert microseconds to seconds */
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);
      
      uint64_t milliseconds = (s_log_buffer[i].timestamp % 1000000) / 1000; /* Milliseconds */
      
      /* Calculate the size of this log entry */
      char timestamp[64];
      snprintf(timestamp, sizeof(timestamp), 
               TIMESTAMP_FORMAT,
               FORMAT_DATE_ARGS(&timeinfo),
               FORMAT_TIME_ARGS(&timeinfo),
               milliseconds);
      
      total_size += strlen(timestamp) + strlen(level_str) + strlen(s_log_buffer[i].buffer) + 10; /* Extra for brackets, spaces, newline */
    }
    
    /* Allocate buffer for all logs */
    all_logs = malloc(total_size + 1); /* +1 for null terminator */
    if (!all_logs) {
      log_error(log_storage_tag, "Memory Error", "Failed to allocate buffer for log compression");
      return ESP_FAIL;
    }
    
    /* Reset buffer position */
    size_t pos = 0;
    
    /* Collect all logs into the buffer */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char *level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
      /* Format: [TIMESTAMP] [LEVEL] MESSAGE */
      time_t log_time = s_log_buffer[i].timestamp / 1000000; /* Convert microseconds to seconds */
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);
      
      uint64_t milliseconds = (s_log_buffer[i].timestamp % 1000000) / 1000; /* Milliseconds */
      
      /* Format the timestamp and log entry */
      int written = snprintf(all_logs + pos, total_size - pos, 
                 TIMESTAMP_FORMAT " [%s] %s\n",
                 FORMAT_DATE_ARGS(&timeinfo),
                 FORMAT_TIME_ARGS(&timeinfo),
                 milliseconds,
                 level_str,
                 s_log_buffer[i].buffer);
      
      if (written > 0) {
        pos += written;
      }
    }
    
    /* Ensure null termination */
    all_logs[pos] = '\0';
    
    /* Write all logs at once */
    esp_err_t ret = priv_write_log_data(s_current_log_file, all_logs);
    free(all_logs);
    
    if (ret != ESP_OK) {
      log_error(log_storage_tag, "Write Failed", "Failed to write compressed logs: %s", 
                esp_err_to_name(ret));
      return ESP_FAIL;
    }
  } else {
    /* No compression, write each log entry individually */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char *level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
      /* Format: [TIMESTAMP] [LEVEL] MESSAGE */
      char formatted_log[MAX_DATA_LENGTH * 2]; /* Doubled the size to ensure enough space */
      time_t log_time = s_log_buffer[i].timestamp / 1000000; /* Convert microseconds to seconds */
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);
      
      uint64_t milliseconds = (s_log_buffer[i].timestamp % 1000000) / 1000; /* Milliseconds */
      
      /* Format the timestamp and log entry */
      snprintf(formatted_log, sizeof(formatted_log), 
               TIMESTAMP_FORMAT " [%s] %s",
               FORMAT_DATE_ARGS(&timeinfo),
               FORMAT_TIME_ARGS(&timeinfo),
               milliseconds,
               level_str,
               s_log_buffer[i].buffer);
      
      /* Enqueue the log for writing */
      esp_err_t ret = file_write_enqueue(s_current_log_file, formatted_log);
      if (ret != ESP_OK) {
        log_error(log_storage_tag, "Write Failed", "Failed to enqueue log for writing: %s", 
                  esp_err_to_name(ret));
        return ESP_FAIL;
      }
    }
  }
  
  /* Reset buffer index */
  s_log_buffer_index = 0;
  log_info(log_storage_tag, "Buffer Flushed", "Successfully flushed log buffer to SD card");
  
  return ESP_OK;
}

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log storage system
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_init(void)
{
  ESP_LOGI(log_storage_tag, "Initializing log storage");
  
  /* Check if already initialized */
  if (s_log_storage_initialized) {
    ESP_LOGW(log_storage_tag, "Log storage already initialized");
    return ESP_OK;
  }
  
  /* Create mutex for thread safety */
  s_log_mutex = xSemaphoreCreateMutex();
  if (s_log_mutex == NULL) {
    ESP_LOGE(log_storage_tag, "Failed to create mutex");
    return ESP_FAIL;
  }
  
  /* Initialize log buffer */
  memset(s_log_buffer, 0, sizeof(s_log_buffer));
  s_log_buffer_index = 0;
  
  /* Set default compression state */
  s_compression_enabled = (log_compression_enabled != 0);
  
  /* Mark as initialized */
  s_log_storage_initialized = true;
  
  ESP_LOGI(log_storage_tag, "Log storage initialized successfully");
  return ESP_OK;
}

esp_err_t log_storage_set_sd_available(bool available)
{
  if (!s_log_storage_initialized) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(log_storage_tag, "Mutex Error", "Failed to acquire mutex for SD card status update");
    return ESP_FAIL;
  }
  
  bool previous_state = s_sd_card_available;
  s_sd_card_available = available;
  
  if (!previous_state && available) {
    /* SD card became available, try to flush buffer */
    log_info(log_storage_tag, "SD Available", "SD card became available, flushing buffered logs");
    priv_flush_log_buffer();
  } else if (previous_state && !available) {
    log_warn(log_storage_tag, "SD Unavailable", "SD card became unavailable, logs will be buffered");
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_write(esp_log_level_t level, const char *message)
{
  if (!s_log_storage_initialized) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    /* Can't log this error through normal channels as it might cause recursion */
    ESP_LOGE(log_storage_tag, "Failed to acquire mutex for log write");
    return ESP_FAIL;
  }
  
  /* Store log in buffer */
  if (s_log_buffer_index < LOG_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strncpy(s_log_buffer[s_log_buffer_index].buffer, message, LOG_STORAGE_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[s_log_buffer_index].buffer[LOG_STORAGE_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++;
  } else {
    /* Buffer is full, need to flush */
    log_warn(log_storage_tag, "Buffer Full", "Log buffer full, forcing flush");
    priv_flush_log_buffer();
    
    /* Store the current log */
    s_log_buffer[0].level     = level;
    s_log_buffer[0].timestamp = esp_timer_get_time();
    strncpy(s_log_buffer[0].buffer, message, LOG_STORAGE_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[0].buffer[LOG_STORAGE_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index = 1;
  }
  
  /* If buffer is full or SD card is available, try to flush */
  if (s_log_buffer_index >= LOG_BUFFER_SIZE && s_sd_card_available) {
    priv_flush_log_buffer();
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_flush(void)
{
  if (!s_log_storage_initialized) {
    log_error(log_storage_tag, "Flush Error", "Log storage not initialized");
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(log_storage_tag, "Mutex Error", "Failed to acquire mutex for log flush");
    return ESP_FAIL;
  }
  
  esp_err_t ret = priv_flush_log_buffer();
  
  xSemaphoreGive(s_log_mutex);
  log_info(log_storage_tag, "Flush Complete", "Log flush completed successfully");
  return ret;
}

esp_err_t log_storage_set_compression(bool enabled)
{
  if (!s_log_storage_initialized) {
    log_error(log_storage_tag, "Config Error", "Log storage not initialized");
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(log_storage_tag, "Mutex Error", "Failed to acquire mutex for compression config");
    return ESP_FAIL;
  }
  
  /* Only update if the setting has changed */
  if (s_compression_enabled != enabled) {
    s_compression_enabled = enabled;
    log_info(log_storage_tag, "Compression Config", "Log compression %s", 
             enabled ? "enabled" : "disabled");
    
    /* Flush current logs before changing compression setting */
    priv_flush_log_buffer();
    
    /* Force log rotation on next write to use new extension */
    s_current_log_file[0] = '\0';
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

bool log_storage_is_compression_enabled(void)
{
  return s_compression_enabled;
} 
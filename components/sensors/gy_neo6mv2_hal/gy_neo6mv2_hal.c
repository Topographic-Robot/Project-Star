/* components/sensors/gy_neo6mv2_hal/gy_neo6mv2_hal.c */

#include "gy_neo6mv2_hal.h"
#include <string.h>
#include <stdlib.h>
#include "esp_err.h"
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/uart.h"
#include "driver/gpio.h"
#include "error_handler.h"
#include "log_handler.h"

/* Constants *******************************************************************/

/* Module constants */
const char       *gy_neo6mv2_tag                    = "GY-NEO6MV2";
const uint8_t     gy_neo6mv2_tx_io                  = GPIO_NUM_17;
const uint8_t     gy_neo6mv2_rx_io                  = GPIO_NUM_16;
const uart_port_t gy_neo6mv2_uart_num               = UART_NUM_2;
const uint32_t    gy_neo6mv2_uart_baudrate          = 9600;
const uint32_t    gy_neo6mv2_polling_rate_ticks     = pdMS_TO_TICKS(5 * 100);
const uint8_t     gy_neo6mv2_max_retries            = 4;
const uint32_t    gy_neo6mv2_initial_retry_interval = pdMS_TO_TICKS(15 * 1000);
const uint32_t    gy_neo6mv2_max_backoff_interval   = pdMS_TO_TICKS(480 * 1000);
const size_t      gy_neo6mv2_rx_buffer_size         = 1024 * 2; /* 2KB RX buffer */
const size_t      gy_neo6mv2_tx_buffer_size         = 1024;     /* 1KB TX buffer */

/* GPGSV sentence parsing constants */
const uint8_t gy_neo6mv2_gpgsv_sats_per_sentence = 4;  /**< Number of satellites per GPGSV sentence */
const uint8_t gy_neo6mv2_gpgsv_field_start       = 4;  /**< Starting field index for satellite data */
const uint8_t gy_neo6mv2_gpgsv_field_step        = 4;  /**< Number of fields per satellite */

/* Globals (Static) ***********************************************************/

static char            s_gy_neo6mv2_sentence_buffer[GY_NEO6MV2_SENTENCE_BUFFER_SIZE]; /**< Buffer for assembling fragmented NMEA sentences received from the GPS module. */
static satellite_t     s_gy_neo6mv2_satellites[GY_NEO6MV2_MAX_SATELLITES];            /**< Buffer to store parsed satellite information from GPGSV sentences. */
static uint32_t        s_gy_neo6mv2_sentence_index  = 0;                              /**< Index tracking the current position in the sentence buffer. */
static uint8_t         s_gy_neo6mv2_satellite_count = 0;                              /**< Counter for the number of satellites currently stored in the buffer. */
static error_handler_t s_gy_neo6mv2_error_handler   = { 0 };

/* Static (Private) Functions *************************************************/

/**
 * @brief Converts a GPS coordinate from NMEA format to decimal degrees.
 *
 * Parses a coordinate in the NMEA format (DDMM.MMMM) used by the GY-NEO6MV2 GPS module 
 * and converts it to decimal degrees. Adjusts the result based on the specified hemisphere.
 *
 * @param[in] coord_str  Pointer to a string containing the coordinate in NMEA format.
 * @param[in] hemisphere Pointer to a character indicating the hemisphere ('N', 'S', 'E', 'W').
 *
 * @return The coordinate in decimal degrees.
 */
static float priv_gy_neo6mv2_parse_coordinate(const char *coord_str, 
                                              const char *hemisphere)
{
  float    coord           = atof(coord_str);
  uint32_t degrees         = (uint32_t)(coord / 100);
  float    minutes         = coord - (degrees * 100);
  float    decimal_degrees = degrees + (minutes / 60.0);

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
 * @brief Validates the checksum of an NMEA sentence.
 *
 * Computes the checksum for an NMEA sentence and compares it with the checksum 
 * provided in the sentence to verify its validity.
 *
 * @param[in] sentence Pointer to a null-terminated NMEA sentence string.
 *
 * @return 
 * - `true`  if the checksum is valid.
 * - `false` otherwise.
 */
static bool priv_gy_neo6mv2_validate_nmea_checksum(const char *sentence)
{
  if (!sentence || sentence[0] != '$') {
    return false;
  }

  const char *checksum_start = strchr(sentence, '*');
  if (!checksum_start || (checksum_start - sentence) > strlen(sentence)) {
    return false;
  }

  uint8_t calculated_checksum = 0;
  for (const char *p = sentence + 1; p < checksum_start; p++) {
    calculated_checksum ^= *p;
  }

  uint8_t sent_checksum = (uint8_t)strtol(checksum_start + 1, NULL, 16);
  return calculated_checksum == sent_checksum;
}

/**
 * @brief Splits an NMEA sentence into individual fields.
 *
 * Tokenizes an NMEA sentence using commas as delimiters and stores the extracted 
 * fields in the provided array. Unused fields are set to `NULL`.
 *
 * @param[in,out] sentence   Pointer to the NMEA sentence (modified in place).
 * @param[out]    fields     Array of pointers to store the extracted fields.
 * @param[in]     max_fields Maximum number of fields to extract.
 */
static void priv_gy_neo6mv2_split_nmea_sentence(char  *sentence, 
                                                char **fields, 
                                                size_t max_fields)
{
  if (!sentence || !fields) {
    return;
  }

  size_t index = 0;
  char  *token = strtok(sentence, ",");
  while (token != NULL && index < max_fields) {
    fields[index++] = token;
    token           = strtok(NULL, ",");
  }

  for (; index < max_fields; index++) {
    fields[index] = NULL;
  }
}

/**
 * @brief Adds satellite data to the buffer.
 *
 * Stores a satellite's information in the buffer if space is available. Logs a 
 * warning and discards the data if the buffer is full.
 *
 * @param[in] prn       Satellite ID (PRN).
 * @param[in] elevation Satellite elevation in degrees.
 * @param[in] azimuth   Satellite azimuth in degrees.
 * @param[in] snr       Signal-to-Noise Ratio (SNR).
 */
static void priv_gy_neo6mv2_add_satellite(uint8_t  prn, 
                                          uint8_t  elevation, 
                                          uint16_t azimuth, 
                                          uint8_t  snr)
{
  if (s_gy_neo6mv2_satellite_count < GY_NEO6MV2_MAX_SATELLITES) {
    satellite_t *sat = &(s_gy_neo6mv2_satellites[s_gy_neo6mv2_satellite_count]);
    sat->prn         = prn;
    sat->elevation   = elevation;
    sat->azimuth     = azimuth;
    sat->snr         = snr;
    s_gy_neo6mv2_satellite_count++;

    log_info(gy_neo6mv2_tag, 
             "Satellite Added", 
             "PRN=%u, Elevation=%u°, Azimuth=%u°, SNR=%u",
             prn, 
             elevation, 
             azimuth, 
             snr);
  } else {
    log_warn(gy_neo6mv2_tag, 
             "Buffer Full", 
             "Cannot add satellite PRN=%u, maximum capacity reached", 
             prn);
  }
}

/**
 * @brief Clears all data from the satellite buffer.
 *
 * Resets the satellite buffer by removing all stored data and setting the 
 * satellite count to zero.
 */
static void priv_gy_neo6mv2_clear_satellites(void)
{
  s_gy_neo6mv2_satellite_count = 0;
  memset(s_gy_neo6mv2_satellites, 0, sizeof(s_gy_neo6mv2_satellites));
  log_info(gy_neo6mv2_tag, "Buffer Cleared", "Satellite buffer has been reset");
}

/**
 * @brief Retrieves satellite data from the buffer.
 *
 * Copies satellite data from the internal buffer into the provided output buffer 
 * for external processing.
 *
 * @param[out] satellites Pointer to the buffer where satellite data will be stored.
 * @param[in]  max_count  Maximum number of satellites to copy.
 *
 * @return The number of satellites copied into the output buffer.
 */
static uint8_t priv_gy_neo6mv2_get_satellites(satellite_t *satellites, 
                                              uint8_t      max_count)
{
  uint8_t count = (s_gy_neo6mv2_satellite_count < max_count) ?
                   s_gy_neo6mv2_satellite_count : max_count;

  memcpy(satellites, s_gy_neo6mv2_satellites, count * sizeof(satellite_t));
  log_info(gy_neo6mv2_tag, 
           "Data Retrieved", 
           "Copied %u satellites from buffer", 
           count);
  return count;
}

/* Public Functions ***********************************************************/

char *gy_neo6mv2_data_to_json(const gy_neo6mv2_data_t *gy_neo6mv2_data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    log_error(gy_neo6mv2_tag, 
              "JSON Creation Failed", 
              "Unable to allocate memory for JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "gps")) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "latitude", gy_neo6mv2_data->latitude)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add latitude field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "longitude", gy_neo6mv2_data->longitude)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add longitude field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "speed", gy_neo6mv2_data->speed)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add speed field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "time", gy_neo6mv2_data->time)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add time field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "fix_status", gy_neo6mv2_data->fix_status)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add fix_status field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "satellite_count", gy_neo6mv2_data->satellite_count)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add satellite_count field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "hdop", gy_neo6mv2_data->hdop)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add hdop field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "retry_count", gy_neo6mv2_data->retry_count)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add retry_count field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "retry_interval", gy_neo6mv2_data->retry_interval)) {
    log_error(gy_neo6mv2_tag, 
              "JSON Field Error", 
              "Failed to add retry_interval field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(gy_neo6mv2_tag, 
              "JSON Serialization Failed", 
              "Unable to convert JSON object to string format");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t gy_neo6mv2_init(void *sensor_data)
{
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  log_info(gy_neo6mv2_tag, 
           "Init Started", 
           "Beginning GY-NEO6MV2 GPS module initialization");

  /* TODO: Initialize error handler */

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, 
                                 gy_neo6mv2_rx_io, 
                                 gy_neo6mv2_uart_baudrate,
                                 gy_neo6mv2_uart_num, 
                                 gy_neo6mv2_rx_buffer_size, 
                                 gy_neo6mv2_tx_buffer_size, 
                                 gy_neo6mv2_tag);
  if (ret != ESP_OK) {
    log_error(gy_neo6mv2_tag, 
              "UART Init Failed", 
              "Failed to initialize UART communication");
    return ret;
  }

  /* Configure GPS module with optimal settings */
  const char *config_commands[] = {
    "$PUBX,41,1,0007,0003,9600,0*10\r\n", /* Set UART1 baud rate */
    "$PUBX,40,GLL,0,0,0,0*5C\r\n",        /* Disable GLL messages */
    "$PUBX,40,GSA,0,0,0,0*4E\r\n",        /* Disable GSA messages */
    "$PUBX,40,GSV,0,0,0,0*59\r\n",        /* Disable GSV messages */
    "$PUBX,40,VTG,0,0,0,0*5E\r\n",        /* Disable VTG messages */
    "$PUBX,40,RMC,1,0,0,0*46\r\n",        /* Enable RMC messages */
    "$PUBX,40,GGA,1,0,0,0*5B\r\n"         /* Enable GGA messages */
  };

  /* Send configuration commands */
  for (size_t i = 0; i < sizeof(config_commands) / sizeof(config_commands[0]); i++) {
    int32_t bytes_written = 0;
    priv_uart_write((const uint8_t *)config_commands[i], 
                    strlen(config_commands[i]), 
                    &bytes_written, 
                    gy_neo6mv2_uart_num, 
                    gy_neo6mv2_tag);
    vTaskDelay(pdMS_TO_TICKS(100)); /* Wait for command processing */
  }

  /* Allow time for the GPS module to warm up and apply settings */
  vTaskDelay(pdMS_TO_TICKS(5000));

  /* Initialize GPS gy_neo6mv2_data fields */
  gy_neo6mv2_data->latitude           = 0.0;                               /* Default latitude */
  gy_neo6mv2_data->longitude          = 0.0;                               /* Default longitude */
  gy_neo6mv2_data->speed              = 0.0;                               /* Default speed */
  gy_neo6mv2_data->fix_status         = 0;                                 /* No fix initially */
  gy_neo6mv2_data->satellite_count    = 0;                                 /* No satellites initially */
  gy_neo6mv2_data->hdop               = 99.99;                             /* Default HDOP value */
  gy_neo6mv2_data->state              = k_gy_neo6mv2_uninitialized;        /* Initial state */
  gy_neo6mv2_data->retry_count        = 0;                                 /* Reset retry count */
  gy_neo6mv2_data->retry_interval     = gy_neo6mv2_initial_retry_interval; /* Default retry interval */
  gy_neo6mv2_data->last_attempt_ticks = 0;                                 /* Reset last attempt ticks */
  memset(gy_neo6mv2_data->time, 0, sizeof(gy_neo6mv2_data->time));         /* Clear time field */

  log_info(gy_neo6mv2_tag, 
           "Init Complete", 
           "GPS module initialization completed successfully");
  return ESP_OK;
}

esp_err_t gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data)
{
  uint8_t uart_rx_buffer[GY_NEO6MV2_SENTENCE_BUFFER_SIZE];
  int32_t length = 0;

  /* Read from UART */
  esp_err_t ret = priv_uart_read(uart_rx_buffer, 
                                 sizeof(uart_rx_buffer),
                                 &length, 
                                 gy_neo6mv2_uart_num, 
                                 gy_neo6mv2_tag);

  if (ret == ESP_OK && length > 0) {
    for (uint32_t i = 0; i < length; i++) {
      char c = uart_rx_buffer[i];

      /* Accumulate characters in the sentence buffer */
      if (s_gy_neo6mv2_sentence_index < sizeof(s_gy_neo6mv2_sentence_buffer) - 1) {
        s_gy_neo6mv2_sentence_buffer[s_gy_neo6mv2_sentence_index++] = c;
      }

      /* Process a complete sentence (ends with \n) */
      if (c == '\n') {
        s_gy_neo6mv2_sentence_buffer[s_gy_neo6mv2_sentence_index] = '\0'; /* Null-terminate */

        /* Strip trailing \r and \n characters */
        while (s_gy_neo6mv2_sentence_index > 0 &&
            (s_gy_neo6mv2_sentence_buffer[s_gy_neo6mv2_sentence_index - 1] == '\r' ||
             s_gy_neo6mv2_sentence_buffer[s_gy_neo6mv2_sentence_index - 1] == '\n')) {
          s_gy_neo6mv2_sentence_buffer[--s_gy_neo6mv2_sentence_index] = '\0';
        }

        /* Log raw sentence */
        log_debug(gy_neo6mv2_tag, 
                  "NMEA Data", 
                  "Received sentence: %s", 
                  s_gy_neo6mv2_sentence_buffer);

        /* Validate checksum */
        if (!priv_gy_neo6mv2_validate_nmea_checksum(s_gy_neo6mv2_sentence_buffer)) {
          log_error(gy_neo6mv2_tag, 
                    "Checksum Error", 
                    "Invalid NMEA sentence checksum: %s", 
                    s_gy_neo6mv2_sentence_buffer);
          s_gy_neo6mv2_sentence_index = 0;
          continue;
        }

        /* Parse specific sentences */
        if (strstr(s_gy_neo6mv2_sentence_buffer, "$GPRMC") == s_gy_neo6mv2_sentence_buffer) {
          /* Parse GPRMC sentence */
          char *fields[12] = { 0 };
          priv_gy_neo6mv2_split_nmea_sentence(s_gy_neo6mv2_sentence_buffer, fields, 12);

          /* Extract and log status */
          if (fields[2]) {
            const char *status = fields[2];
            log_info(gy_neo6mv2_tag, 
                     "GPS Status", 
                     "Fix status: %s (%s)", 
                     status, 
                     (status[0] == 'A') ? "Fix acquired" : "No fix");

            /* Process only valid readings */
            if (status[0] == 'A') {
              /* Extract latitude, longitude, and other data */
              sensor_data->latitude  = priv_gy_neo6mv2_parse_coordinate(fields[3], fields[4]);
              sensor_data->longitude = priv_gy_neo6mv2_parse_coordinate(fields[5], fields[6]);
              sensor_data->speed     = fields[7] ? atof(fields[7]) : 0.0;

              /* Update valid fix status */
              sensor_data->fix_status = 1; /* Fix acquired */
              strncpy(sensor_data->time, fields[1], sizeof(sensor_data->time) - 1);

              log_info(gy_neo6mv2_tag, 
                       "Position Updated", 
                       "Lat: %.6f°, Lon: %.6f°, Speed: %.2f m/s",
                       sensor_data->latitude, 
                       sensor_data->longitude, 
                       sensor_data->speed);
            } else {
              sensor_data->fix_status = 0; /* No fix */
              log_warn(gy_neo6mv2_tag, 
                       "No Fix", 
                       "Skipping invalid GPS reading");
            }
          }
        } else if (strstr(s_gy_neo6mv2_sentence_buffer, "$GPGSV") == s_gy_neo6mv2_sentence_buffer) {
          /* Parse GPGSV sentence for satellite information */
          char *fields[GY_NEO6MV2_GPGSV_MAX_FIELDS] = { '\0' };
          priv_gy_neo6mv2_split_nmea_sentence(s_gy_neo6mv2_sentence_buffer, 
                                              fields, 
                                              GY_NEO6MV2_GPGSV_MAX_FIELDS);

          uint8_t total_sentences  = fields[1] ? (uint8_t)atoi(fields[1]) : 0;
          uint8_t sentence_number  = fields[2] ? (uint8_t)atoi(fields[2]) : 0;
          uint8_t total_satellites = fields[3] ? (uint8_t)atoi(fields[3]) : 0;

          log_info(gy_neo6mv2_tag, 
                   "Satellite Info", 
                   "Sentence %u of %u, Total satellites in view: %u", 
                   sentence_number, 
                   total_sentences, 
                   total_satellites);

          /* Clear satellite data if this is the first sentence */
          if (sentence_number == 1) {
            priv_gy_neo6mv2_clear_satellites();
          }

          /* Parse satellite details (up to 4 satellites per GPGSV sentence) */
          for (uint8_t i = gy_neo6mv2_gpgsv_field_start; i < GY_NEO6MV2_GPGSV_MAX_FIELDS; i += gy_neo6mv2_gpgsv_field_step) {
            if (fields[i] && fields[i + 1] && fields[i + 2] && fields[i + 3]) {
              uint8_t  prn       = (uint8_t)atoi(fields[i]);      /* Satellite ID (PRN) */
              uint8_t  elevation = (uint8_t)atoi(fields[i + 1]);  /* Elevation in degrees */
              uint16_t azimuth   = (uint16_t)atoi(fields[i + 2]); /* Azimuth in degrees */
              uint8_t  snr       = (uint8_t)atoi(fields[i + 3]);  /* SNR (Signal-to-Noise Ratio) */

              /* Store satellite data */
              priv_gy_neo6mv2_add_satellite(prn, elevation, azimuth, snr);
            }
          }
        }

        /* Reset buffer for next sentence */
        s_gy_neo6mv2_sentence_index = 0;
      }
    }

    /* After processing sentences, retrieve satellite data */
    satellite_t local_satellites[GY_NEO6MV2_MAX_SATELLITES];
    uint8_t     satellite_count = priv_gy_neo6mv2_get_satellites(local_satellites, 
                                                                 GY_NEO6MV2_MAX_SATELLITES);

    /* Update sensor_data with satellite count */
    sensor_data->satellite_count = satellite_count;

    /* Process satellite data as needed */
    for (uint8_t i = 0; i < satellite_count; i++) {
      log_info(gy_neo6mv2_tag, 
               "Satellite Details", 
               "Retrieved satellite PRN=%u, Elevation=%u°, Azimuth=%u°, SNR=%u",
               local_satellites[i].prn, 
               local_satellites[i].elevation,
               local_satellites[i].azimuth, 
               local_satellites[i].snr);
    }

    return ESP_OK;
  } else {
    log_error(gy_neo6mv2_tag, 
              "Read Failed", 
              "Unable to read data from GPS module");
    sensor_data->state = k_gy_neo6mv2_error;
    return ESP_FAIL;
  }
}

void gy_neo6mv2_reset_on_error(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data->state == k_gy_neo6mv2_error) {
    /* TODO: Reset error handler */
  }
}

void gy_neo6mv2_tasks(void *sensor_data)
{
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  while (1) {
    if (gy_neo6mv2_read(gy_neo6mv2_data) == ESP_OK) {
      char *json = gy_neo6mv2_data_to_json(gy_neo6mv2_data);
      send_sensor_data_to_webserver(json);
      file_write_enqueue("gy_neo6mv2.txt", json);
      free(json);
    } else {
      gy_neo6mv2_reset_on_error(gy_neo6mv2_data);
    }
    vTaskDelay(gy_neo6mv2_polling_rate_ticks);
  }
}



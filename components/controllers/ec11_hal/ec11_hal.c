/* components/controllers/ec11_hal/ec11_hal.c */

/* TODO: Integrate the MCP23018 */
/* TODO: Test */

#include "ec11_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const char    *ec11_tag                  = "EC11";
const uint32_t ec11_button_debounce_ms   = 50;
const uint32_t ec11_rotation_debounce_ms = 5;

/* Private Functions **********************************************************/

static void process_encoder_state(ec11_data_t *encoder, int current_state) 
{
  int64_t current_time = esp_timer_get_time() / 1000; /* Convert to milliseconds */
  
  /* Check if enough time has passed since last rotation */
  if ((current_time - encoder->last_rotation_time) < ec11_rotation_debounce_ms) {
    return;
  }

  /* Determine the direction of rotation */
  if ((encoder->prev_state == k_ec11_state_00 && current_state == k_ec11_state_01) ||
      (encoder->prev_state == k_ec11_state_01 && current_state == k_ec11_state_11) ||
      (encoder->prev_state == k_ec11_state_11 && current_state == k_ec11_state_10) ||
      (encoder->prev_state == k_ec11_state_10 && current_state == k_ec11_state_00)) {
    encoder->position++;
    encoder->last_rotation_time = current_time;
    if (encoder->callback) {
      encoder->callback(k_ec11_cw, encoder->board_ptr, encoder->motor_mask);
    }
  } else if ((encoder->prev_state == k_ec11_state_00 && current_state == k_ec11_state_10) ||
             (encoder->prev_state == k_ec11_state_10 && current_state == k_ec11_state_11) ||
             (encoder->prev_state == k_ec11_state_11 && current_state == k_ec11_state_01) ||
             (encoder->prev_state == k_ec11_state_01 && current_state == k_ec11_state_00)) {
    encoder->position--;
    encoder->last_rotation_time = current_time;
    if (encoder->callback) {
      encoder->callback(k_ec11_ccw, encoder->board_ptr, encoder->motor_mask);
    }
  }
  encoder->prev_state = current_state;
}

static void process_button_state(ec11_data_t *encoder, bool current_button) 
{
  int64_t current_time = esp_timer_get_time() / 1000; /* Convert to milliseconds */
  
  /* Check if enough time has passed since last button event */
  if ((current_time - encoder->last_button_time) < ec11_button_debounce_ms) {
    return;
  }

  if (current_button != encoder->button_pressed) {
    encoder->button_pressed   = current_button;
    encoder->last_button_time = current_time;
    if (encoder->callback) {
      encoder->callback(current_button ? k_ec11_btn_press : k_ec11_btn_release,
                       encoder->board_ptr,
                       encoder->motor_mask);
    }
  }
}

/* Public Functions ***********************************************************/

esp_err_t ec11_init(ec11_data_t *encoder)
{
  if (!encoder) {
    log_error(ec11_tag, "Init Error", "Encoder pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(ec11_tag, "Init Start", "Beginning EC11 rotary encoder initialization");

  /* Configure GPIO pins */
  gpio_config_t io_conf = {
    .pin_bit_mask = ((uint64_t)1 << encoder->pin_a) |
                    ((uint64_t)1 << encoder->pin_b) |
                    ((uint64_t)1 << encoder->pin_btn),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_ANYEDGE
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_error(ec11_tag, "GPIO Error", "Failed to configure pins A:%u, B:%u, BTN:%u", 
              encoder->pin_a, encoder->pin_b, encoder->pin_btn);
    encoder->state = k_ec11_error;
    return ret;
  }

  /* Initialize encoder state */
  encoder->position           = 0;
  encoder->button_pressed     = false;
  encoder->prev_state         = (gpio_get_level(encoder->pin_a) << 1) |
                                gpio_get_level(encoder->pin_b);
  encoder->state              = k_ec11_ready;
  encoder->last_button_time   = 0;
  encoder->last_rotation_time = 0;

  /* Create mutex for thread-safe access in ISR */
  encoder->mutex = xSemaphoreCreateMutex();
  if (!encoder->mutex) {
    log_error(ec11_tag, "Mutex Error", "Failed to create mutex for encoder");
    encoder->state = k_ec11_error;
    return ESP_ERR_NO_MEM;
  }

  /* Install GPIO ISR service and handler */
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    log_error(ec11_tag, "ISR Error", "Failed to install GPIO ISR service");
    encoder->state = k_ec11_error;
    return ret;
  }

  /* Add ISR handler for pin A */
  ret = gpio_isr_handler_add(encoder->pin_a, ec11_isr_handler, (void *)encoder);
  if (ret != ESP_OK) {
    log_error(ec11_tag, "ISR Error", "Failed to add ISR handler for pin A");
    encoder->state = k_ec11_error;
    return ret;
  }

  /* Add ISR handler for pin B */
  ret = gpio_isr_handler_add(encoder->pin_b, ec11_isr_handler, (void *)encoder);
  if (ret != ESP_OK) {
    log_error(ec11_tag, "ISR Error", "Failed to add ISR handler for pin B");
    encoder->state = k_ec11_error;
    return ret;
  }

  /* Add ISR handler for button pin */
  ret = gpio_isr_handler_add(encoder->pin_btn, ec11_isr_handler, (void *)encoder);
  if (ret != ESP_OK) {
    log_error(ec11_tag, "ISR Error", "Failed to add ISR handler for button pin");
    encoder->state = k_ec11_error;
    return ret;
  }

  encoder->position       = 0;
  encoder->button_pressed = false;
  log_info(ec11_tag, "Init Complete", "EC11 encoder initialized successfully");
  return ESP_OK;
}

void ec11_register_callback(ec11_data_t  *encoder,
                          ec11_callback_t callback,
                          void           *board_ptr,
                          uint16_t        motor_mask)
{
  if (encoder == NULL || callback == NULL || board_ptr == NULL) {
    log_error(ec11_tag, "Callback Error", "Invalid parameters: encoder=%p, callback=%p, board=%p",
              (void*)encoder, (void*)callback, board_ptr);
    return;
  }

  if (xSemaphoreTake(encoder->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(ec11_tag, "Mutex Error", "Failed to acquire mutex for callback registration");
    return;
  }
  
  encoder->callback   = callback;
  encoder->board_ptr  = board_ptr;
  encoder->motor_mask = motor_mask;
  log_info(ec11_tag, "Callback Set", "Registered callback for encoder with motor mask 0x%04X", motor_mask);
  xSemaphoreGive(encoder->mutex);
}

void IRAM_ATTR ec11_isr_handler(void *arg)
{
  ec11_data_t *encoder                  = (ec11_data_t *)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (xSemaphoreTakeFromISR(encoder->mutex, &higher_priority_task_woken) == pdTRUE) {
    /* Read current state */
    int current_state   = (gpio_get_level(encoder->pin_a) << 1) |
                          gpio_get_level(encoder->pin_b);
    bool current_button = gpio_get_level(encoder->pin_btn) == 0; /* Active low */

    /* Process encoder and button states */
    if (current_state != encoder->prev_state) {
      process_encoder_state(encoder, current_state);
    }
    process_button_state(encoder, current_button);

    xSemaphoreGiveFromISR(encoder->mutex, &higher_priority_task_woken);
  }

  /* Yield if a higher priority task was woken */
  if (higher_priority_task_woken) {
    portYIELD_FROM_ISR();
  }
}

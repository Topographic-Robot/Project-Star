/* main/gait_movement.c */

#include "gait_movement.h"
#include <math.h>
#include "hexapod_geometry.h"
#include "pca9685_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const char *gait_tag = "Gait Movement";

/* Globals (Static) ***********************************************************/

static leg_t s_legs[num_legs] = {};

/* Private Functions (Static) *************************************************/

/**
 * @brief Configures a motor with joint type, board ID, and motor ID.
 *
 * Assigns joint type, board ID, motor ID, and initializes the motor's position 
 * to 0.0 degrees. Adjusts motor indices to wrap correctly across multiple 
 * PCA9685 boards when needed.
 *
 * @param[in,out] motor       Pointer to the `motor_t` structure to configure.
 * @param[in]     joint_type  Type of joint (e.g., hip, knee, tibia).
 * @param[in]     board       Pointer to the current PCA9685 board.
 * @param[in,out] motor_index Pointer to the motor index on the current board.
 * @param[in,out] board_id    Pointer to the current PCA9685 board ID.
 *
 * @note 
 * - Updates `motor_index` and `board_id` when indices exceed the board's capacity (16 motors).
 * - Returns an error if a new board is required but unavailable.
 *
 * @return 
 * - `ESP_OK`   on success.
 * - `ESP_FAIL` if a new board is required but unavailable.
 */
static esp_err_t priv_assign_motor(motor_t *motor, joint_type_t joint_type, 
                                   pca9685_board_t **board, uint8_t *motor_index, 
                                   uint8_t *board_id)
{
  if (*motor_index >= 16) {
    if ((*board)->next == NULL) {
      ESP_LOGE(gait_tag, "Insufficient PCA9685 boards for initializing motors.");
      return ESP_FAIL;
    }
    *board       = (*board)->next;
    *motor_index = 0;
    (*board_id)++;
  }

  motor->joint_type = joint_type;
  motor->pos_deg    = 0.0f;
  motor->board_id   = *board_id;
  motor->motor_id   = *motor_index;

  (*motor_index)++;
  return ESP_OK;
}

/* TODO: Doc Comment */
esp_err_t priv_move_motors(pca9685_board_t *pwm_controller, uint32_t motor_mask, 
                           float angle)
{
  uint16_t lower_16_mask = motor_mask & 0xFFFF;         /* Extract the lower 16 bits */
  uint16_t upper_16_mask = (motor_mask >> 16) & 0xFFFF; /* Extract the upper 16 bits */

  /* Attempt to set the angle for the lower 16 motors */
  esp_err_t ret = pca9685_set_angle(pwm_controller, lower_16_mask, 0, angle);
  if (ret != ESP_OK) {
    ESP_LOGE(gait_tag, "Failed to set angle for lower 16 motors");
    return ret;
  }

  /* Attempt to set the angle for the upper 16 motors */
  ret = pca9685_set_angle(pwm_controller, upper_16_mask, 1, angle);
  if (ret != ESP_OK) {
    ESP_LOGE(gait_tag, "Failed to set angle for upper 16 motors");
    return ret;
  }

  return ESP_OK;
}

/* TODO: Doc Comment */
inline void priv_add_motor_to_mask(uint32_t *mask, motor_t motor)
{
  *mask |= (1 << motor.motor_id) << (motor.board_id * 16);
}

/* TODO: Doc Comment */
esp_err_t priv_standup(pca9685_board_t *pwm_controller)
{
  /* bottom left, top right, bottom right, top left, middle left, middle right */
  const uint8_t leg_order[num_legs]      = {0, 5, 1, 4, 2, 3};
  const float   default_knee_pos_from90  = -30; /* TODO: find the proper degree, -30 is just a place holder */
  const float   default_tibia_pos_from90 = -30; /* TODO: find the proper degree, -30 is just a place holder */
  const float   default_hip_pos_from90   = -30; /* TODO: find the proper degree, -30 is just a place holder */

  for (uint8_t i = 0; i < num_legs; i += max_active_servos) {
    uint32_t motor_mask = 0;

    /* Process knee movement in chunks of max_active_servos */
    for (uint8_t j = 0; j < max_active_servos && (i + j) < num_legs; j++) {
      priv_add_motor_to_mask(&motor_mask, *(s_legs[leg_order[i + j]].knee_motor));
    }
    priv_move_motors(pwm_controller, motor_mask, default_knee_pos_from90);
    vTaskDelay(pdMS_TO_TICKS(100));

    motor_mask = 0; /* reset mask */

    /* Process tibia movement in chunks of max_active_servos */
    for (uint8_t j = 0; j < max_active_servos && (i + j) < num_legs; j++) {
      priv_add_motor_to_mask(&motor_mask, *(s_legs[leg_order[i + j]].tibia_motor));
    }
    priv_move_motors(pwm_controller, motor_mask, default_tibia_pos_from90);
    vTaskDelay(pdMS_TO_TICKS(100));

    motor_mask = 0; /* reset mask */

    /* Process hip movement in chunks of max_active_servos */
    for (uint8_t j = 0; j < max_active_servos && (i + j) < num_legs; j++) {
      priv_add_motor_to_mask(&motor_mask, *(s_legs[leg_order[i + j]].hip_motor));
    }
    priv_move_motors(pwm_controller, motor_mask, default_hip_pos_from90);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t tripod_gait(pca9685_board_t *pwm_controller, float heading, 
                      uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting tripod gait: heading=%.2f, distance=%ucm", 
           heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t wave_gait(pca9685_board_t *pwm_controller, float heading, 
                    uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting wave gait: heading=%.2f, distance=%ucm", 
           heading, distance);

  /* To move the knee so it presses on the floor we move it in +deg 
     To move the knee so its go in the air we move it in -deg
     */

  //lift_knee(pwm_controller, deg, leg0)

  /* Move leg 0
     - move knee upwards (lift the foot off the ground)
     - move tibia to angle
     - move hip to angle
     - move knee to angle (this should place it back on the ground)
     */

  return ESP_OK;
}

esp_err_t ripple_gait(pca9685_board_t *pwm_controller, float heading, 
                      uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting ripple gait: heading=%.2f, distance=%ucm", 
           heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t quadruped_gait(pca9685_board_t *pwm_controller, float heading, 
                         uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting quadruped gait: heading=%.2f, distance=%ucm", 
           heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t gait_init(pca9685_board_t *pwm_controller)
{
  if (pwm_controller == NULL) {
    ESP_LOGE(gait_tag, "PCA9685 controller pointer is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize leg configurations */
  uint8_t          motor_index = 0;
  uint8_t          board_id    = 0;
  pca9685_board_t *board       = pwm_controller;

  /* Map motors to s_legs and joints */
  for (uint8_t leg_id = 0; leg_id < num_legs; ++leg_id) {
    s_legs[leg_id].id = leg_id;

    /* Assign motors to each joint */
    esp_err_t ret1, ret2, ret3;
    s_legs[leg_id].hip_motor   = &(board->motors[motor_index]);
    s_legs[leg_id].knee_motor  = &(board->motors[motor_index]);
    s_legs[leg_id].tibia_motor = &(board->motors[motor_index]);

    ret1 = priv_assign_motor(s_legs[leg_id].hip_motor, k_hip, &board, &motor_index, &board_id);
    ret2 = priv_assign_motor(s_legs[leg_id].knee_motor, k_knee, &board, &motor_index, &board_id);
    ret3 = priv_assign_motor(s_legs[leg_id].tibia_motor, k_tibia, &board, &motor_index, &board_id);

    if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK) {
      ESP_LOGE(gait_tag, "Failed to initialize motors for leg %u.", leg_id);
      return ESP_FAIL;
    }
  }

  /* Attempt to make the robot stand up */
  esp_err_t standup_result = priv_standup(pwm_controller);
  if (standup_result != ESP_OK) {
    ESP_LOGE(gait_tag, "Failed to make the robot stand up");
    return standup_result;
  }

  ESP_LOGI(gait_tag, "Legs initialized and robot is in a standing position.");
  return ESP_OK;
}


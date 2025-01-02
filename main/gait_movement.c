/* main/gait_movement.c */

#include "gait_movement.h"
#include <math.h>
#include "hexapod_geometry.h"
#include "pca9685_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Globals (Static) ***********************************************************/

static leg_t s_legs[6] = {};

/* Constants ******************************************************************/

const char *gait_tag = "Gait Movement";

/* Private Functions (Static) *************************************************/

/* TODO: Doc Comment */
esp_err_t priv_move_motors(pca9685_board_t *pwm_controller, 
                           motor_mask, float angle)
{
  uint16_t lower_16_mask = motor_mask & 0xFFFF;          /* Extract the lower 16 bits */
  uint16_t upper_16_mask = (motor_mask >> 16) & 0xFFFF;  /* Extract the upper 16 bits */

  pca9685_set_angle(pwm_controller, lower_16_mask, 0, angle);
  pca9685_set_angle(pwm_controller, upper_16_mask, 1, angle);
}

/* TODO: Doc Comment */
inline void priv_add_motor_to_mask(uint32_t *mask, motor_t motor)
{
  *mask |= (1 << motor.motor_id) << (motor.board_id * 16);
}

/* TODO: Doc Comment */
esp_err_t priv_standup(pca9685_board_t *pwm_controller)
{
  /* ORDER: legs- (0, 5), (1, 4), (2, 3) [(bottom left, top right), (bottom right, top left), (middle left, middle right)] */
  uint8_t leg_pairs[3][2] = {{0, 5}, {1, 4}, {2, 3}};
  for (uint8_t i = 0; i < 3; i++) {
    uint32_t motor_mask = 0;
    priv_add_motor_to_mask(&motor_mask, s_legs[leg_pairs[i][0]].knee_motor);
    priv_add_motor_to_mask(&motor_mask, s_legs[leg_pairs[i][1]].knee_motor);
    priv_move_motors(motor_mask, deg);
  }

  /* Lift the knee */
  /* Move the tibia */
  /* Drop the knee so the foot (end of tibia) hits the floor*/

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

  lift_knee(pwm_controller, deg, leg0)

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
  /* XXX: THIS MUST BE CORRECT WITH THE WIRING BEFORE TESTING ANYTHING OR UH OOOHH */
  /* TODO: MAKE SURE THIS IS RIGHT! IMPORTANT */

  if (pwm_controller == NULL) {
    ESP_LOGE(gait_tag, "PCA9685 controller pointer is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize leg configurations */
  uint8_t          motor_index = 0;
  uint8_t          board_id    = 0;
  pca9685_board_t *board       = pwm_controller;

  /* Map motors to s_legs and joints */
  for (uint8_t leg_id = 0; leg_id < 6; ++leg_id) {
    s_legs[leg_id].id = leg_id;

    /* Assign motors to each joint */
    esp_err_t ret1, ret2, ret3;
    s_legs[leg_id].hip_motor   = &(board->motors[motor_index]);
    s_legs[leg_id].knee_motor  = &(board->motors[motor_index]);
    s_legs[leg_id].tibia_motor = &(board->motors[motor_index]);

    ret1 = priv_assign_motor(s_legs[leg_id].hip_motor, k_hip, &board, &motor_index,
                             &board_id);
    ret2 = priv_assign_motor(s_legs[leg_id].knee_motor, k_knee, &board, &motor_index,
                             &board_id);
    ret3 = priv_assign_motor(s_legs[leg_id].tibia_motor, k_tibia, &board, &motor_index,
                             &board_id);

    if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK) {
      ESP_LOGE(gait_tag, "Failed to initialize motors for leg %u.", leg_id);
      return ESP_FAIL;
    }
  }

  /* TODO: Handle the ret from this func */
  priv_standup(); /* Make the robot stand up */

  ESP_LOGI(gait_tag, "Legs initialized successfully.");
  return ESP_OK;
}

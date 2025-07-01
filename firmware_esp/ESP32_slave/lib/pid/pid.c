#include "pid.h"

const char *TAG_PID = "PID";

pid_ctrl_block_handle_t init_pid(pid_side_t side) {

  pid_ctrl_config_t config_pid; 
  pid_ctrl_block_handle_t pid_block; 

  pid_ctrl_parameter_t values_pid = {
    .kd = PID_SIDE_KD(side),
    .kp = PID_SIDE_KP(side),
    .ki = PID_SIDE_KI(side),
    .min_integral = Min_integral,
    .max_integral = Max_integral,
    .min_output = Min_Output,
    .max_output = Max_Output,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
  };

  config_pid.init_param = values_pid;

  ESP_ERROR_CHECK(pid_new_control_block(&config_pid, &pid_block));
  ESP_ERROR_CHECK(pid_update_parameters(pid_block, &values_pid));

  return pid_block;
}

void PWM_limit(float *PWM) {
  if (*PWM > 1023) {

    *PWM = 1023;

  } else if (*PWM < -1023) {
    
    *PWM = -1023;

  }
}

esp_err_t pid_calculate(pcnt_unit_handle_t upcnt_unit_L, pid_ctrl_block_handle_t pid_block_L, pcnt_unit_handle_t upcnt_unit_R, pid_ctrl_block_handle_t pid_block_R) {
  float controll_pid_LEFT = 0;
  float controll_pid_RIGHT = 0;
  
  //Target values received is represented in RAD/s

   while(1) {

    // ESP_LOGI(TAG_PID, "Target LEFT: %f", TARGET_VALUE_L);
    // ESP_LOGI(TAG_PID, "Target RIGHT: %f", TARGET_VALUE_R);

    //Global variables
    ENCODER_READ_L = pulse_count(upcnt_unit_L);
    ENCODER_READ_R = pulse_count(upcnt_unit_R);

    ESP_LOGI(TAG_PID, "Encoder LEFT: %d", ENCODER_READ_L);
    ESP_LOGI(TAG_PID, "Encoder RIGHT: %d", ENCODER_READ_R);

    RADS_L = ENCODER_READ_L * PID_TICKS_TO_RADS(PID_LEFT);
    RADS_R = ENCODER_READ_R * PID_TICKS_TO_RADS(PID_RIGHT);

    // ESP_LOGI(TAG_PID, "Initial speed LEFT: %f", RADS_L);
    // ESP_LOGI(TAG_PID, "Initial speed  RIGHT: %f", RADS_R);

    if(TARGET_VALUE_L == 0 && TARGET_VALUE_R == 0 && BREAK_FLAG) {
      LEFT_PWM_VALUE = 0;
      RIGHT_PWM_VALUE = 0;
    }

    else{
      float error_motor_LEFT = (TARGET_VALUE_L - RADS_L);
      float error_motor_RIGHT = (TARGET_VALUE_R - RADS_R);
    
      // ESP_LOGI(TAG_PID, "Error LEFT: %f", error_motor_LEFT);
      //  ESP_LOGI(TAG_PID, "Error RIGHT: %f", error_motor_RIGHT);

      // Calculate a new PWM Value
      if(pid_compute(pid_block_L, error_motor_LEFT, &controll_pid_LEFT) != ESP_OK) {
        ESP_LOGE(TAG_PID, "Error computing PID");
      }
      if((pid_compute(pid_block_R, error_motor_RIGHT, &controll_pid_RIGHT)) != ESP_OK) {
        ESP_LOGE(TAG_PID, "Error computing PID");
      }

      // ESP_LOGI(TAG_PID, "PWM command LEFT: %f", controll_pid_LEFT);
      //  ESP_LOGI(TAG_PID, "PWM command RIGHT: %f", controll_pid_RIGHT);

      LEFT_PWM_VALUE += controll_pid_LEFT;
      RIGHT_PWM_VALUE += controll_pid_RIGHT;

      PWM_limit(&LEFT_PWM_VALUE);
      PWM_limit(&RIGHT_PWM_VALUE);
      
      //ESP_LOGI(TAG_PID, "PWM RIGHT: %f", RIGHT_PWM_VALUE);
      // ESP_LOGI(TAG_PID, "PWM LEFT: %f", LEFT_PWM_VALUE);
    }

      update_motor(LEFT, LEFT_PWM_VALUE);
      update_motor(RIGHT, RIGHT_PWM_VALUE);

      controll_pid_LEFT = 0;
      controll_pid_RIGHT = 0;

      vTaskDelay(60 / portTICK_PERIOD_MS);
    
   }

  return ESP_OK;
}
  
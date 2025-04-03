#include "task_manager.h"

void i2c_task_com() {
    i2c_init();

    while(1) {
        i2c_read_task();
        i2c_write_task(ENCODER_READ_L, ENCODER_READ_R);
        vTaskDelay(FREQ_COMMUNICATION / portTICK_PERIOD_MS);
    }
    
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
}

void task_motor_control() {

    init_gpio();
    init_pwm();

    pid_ctrl_block_handle_t pid_block_left = init_pid(PID_LEFT);
    pid_ctrl_block_handle_t pid_block_right = init_pid(PID_RIGHT);
    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1) {
        pid_calculate(encoder_unit_left, pid_block_left, encoder_unit_right, pid_block_right);

        vTaskDelay(2 * 58 / portTICK_PERIOD_MS);
    }
   
}

esp_err_t init_tasks() {

    // Task 1 (core 0): read + write data
    xTaskCreatePinnedToCore(i2c_task_com, "i2c_task_com", 4096, NULL, 1, NULL, 0);

    // Task 2 (core 1): control 
    xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 1);

    return ESP_OK;
}
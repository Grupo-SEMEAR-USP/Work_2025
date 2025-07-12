#include "task_manager.h"



void mqtt_task(void *pv)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    mqtt_start(BROKER_URI);

    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while (1) {
        mqtt_publish_encoders(encoder_unit_right, encoder_unit_left);
        vTaskDelay(FREQ_COMMUNICATION / portTICK_PERIOD_MS);
    }
}


void serial_task(void *pv)
{

    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while (1) {
        uart_comm_read();
        uart_comm_write(encoder_unit_right, encoder_unit_left);
        vTaskDelay(FREQ_COMMUNICATION_SERIAL / portTICK_PERIOD_MS);
    }
}

void i2c_task_com() {
    i2c_init();

    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1) {
        i2c_read_task();
        i2c_write_task(encoder_unit_right, encoder_unit_left);
        vTaskDelay(FREQ_COMMUNICATION / portTICK_PERIOD_MS);
    }
    
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
}

void task_motor_control() {

    init_gpio();
    init_pwm();

    pid_ctrl_block_handle_t pid_block_left = init_pid(PID_LEFT);
    pid_ctrl_block_handle_t pid_block_right = init_pid(PID_RIGHT);

    while(1) {
        pid_calculate(pid_block_left, pid_block_right);

        vTaskDelay(FREQ_TASK_MOTOR / portTICK_PERIOD_MS);
    }
   
}

esp_err_t init_tasks() {

    // Task 1 (core 0): read + write data
    xTaskCreatePinnedToCore(i2c_task_com, "i2c_task_com", 4096, NULL, 1, NULL, 0);

    // Task 2 (core 1): control 
    xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 1);

    return ESP_OK;
}
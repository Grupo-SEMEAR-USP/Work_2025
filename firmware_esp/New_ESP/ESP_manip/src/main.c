#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_manager.h"
#include "servo_control.h"
#include "stepper_control.h"
#include "ultrassonics.h"

static const char* TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(init_tasks());
}
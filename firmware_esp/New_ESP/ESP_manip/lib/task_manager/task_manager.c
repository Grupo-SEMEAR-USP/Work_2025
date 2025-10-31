#include "task_manager.h"
#include "servo_control.h"
#include "stepper_control.h"
#include "mqtt_communication.h"
#include "ultrassonics.h"
#include "terminal_cli.h"
#include "utils.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdbool.h>

static const char *TAG = "tasks";

static inline bool nearly_eq(float a, float b, float eps){ return fabsf(a-b) <= eps; }

static inline int snap_steps(float delta, float deadband) {
    if (fabsf(delta) <= deadband) return 0;
    return (int)lrintf(delta);
}

static void apply_from_globals_once(void)
{
    // 1) STEPPERS

    static float last_arm_sp  = NAN;
    static float last_base_sp = NAN; 

    const float arm_sp = STEPPER_ARM;     
    const float base_sp = STEPPER_ROTATORY_BASE;

    const float step_eps = 0.49f;

    if (isnan(last_arm_sp)) {
        last_arm_sp = arm_sp;
    } else if (!nearly_eq(arm_sp, last_arm_sp, step_eps)) {
        stepper_elevator_move(arm_sp);
        last_arm_sp = arm_sp;
        ESP_LOGI(TAG, "Elevator: passos=%f", arm_sp);
    }

    if (isnan(last_base_sp)) {
        last_base_sp = base_sp;
    } else if (!nearly_eq(base_sp, last_base_sp, step_eps)) {
        stepper_base_move(base_sp);
        last_base_sp = base_sp;
        ESP_LOGI(TAG, "BaseRot: passos=%f", base_sp);
    }

    // 2) SERVOS
    static float last_wrist   = NAN;
    static float last_gripper = NAN;

    float wrist   = fminf(290.f, fmaxf(0.f, SERVO_WRIST));
    float gripper = fminf(290.f, fmaxf(0.f, SERVO_GRIPPER));

    if (isnan(last_wrist) || !nearly_eq(wrist, last_wrist, 1e-3f)) {
        servo_write_wrist(wrist);
        last_wrist = wrist;
        ESP_LOGI(TAG, "Wrist: %.1f°", (double)wrist);
    }
    if (isnan(last_gripper) || !nearly_eq(gripper, last_gripper, 1e-3f)) {
        servo_write_gripper(gripper);
        last_gripper = gripper;
        ESP_LOGI(TAG, "Gripper: %.1f°", (double)gripper);
    }
}

void mqtt_task(void *pv)
{
    wifi_init();
    mqtt_start("mqtt://192.168.1.100:1883");

    ESP_ERROR_CHECK(ultrassonic_init());

    while (1) {
        mqtt_publish_ultrassonics();
        ultrassonic_sample_once();   
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}


void task_manipulator(void *pv)
{
    while (1) {
        apply_from_globals_once();

        stepper_task_loop();

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

esp_err_t init_tasks(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Servos
    ESP_ERROR_CHECK(servo_init());

    // Steppers
    ESP_ERROR_CHECK(stepper_init());

    // (Opcional) CLI no UART
    // start_cli();

    // Tasks
    xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_manipulator, "task_manipulator",  4096, NULL, 5, NULL, 1);

    return ESP_OK;
}

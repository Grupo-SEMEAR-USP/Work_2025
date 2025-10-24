#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_BROKER_URI
#define CONFIG_BROKER_URI "mqtt://192.168.1.100"
#endif

typedef struct {
    float cmd[4];
} control_cmd_t;

esp_err_t init_tasks(void);

void cmds_set(float e_steps, float b_steps, float grip_deg, float wrist_deg);

void cmds_get(control_cmd_t *out);

bool apply_if_changed(void);

void mqtt_task(void *pv);
void task_motor_control(void *pv);

#ifdef __cplusplus
}
#endif

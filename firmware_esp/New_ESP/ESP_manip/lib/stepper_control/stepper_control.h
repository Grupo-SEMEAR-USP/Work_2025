#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_STEP_ELEVATOR 15
#define GPIO_DIR_ELEVATOR 2
#define GPIO_STEP_BASE 14
#define GPIO_DIR_BASE 27

#define MAX_SPEED 1000.f
#define ACCEL 500.f

esp_err_t stepper_init();

void stepper_elevator_move(int steps);
void stepper_base_move(int steps);

void stepper_task_loop(void);

bool stepper_is_idle(void);

#ifdef __cplusplus
}
#endif

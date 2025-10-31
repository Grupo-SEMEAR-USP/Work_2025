#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_WRIST 22
#define GPIO_GRIPPER 32

esp_err_t servo_init();

// Ângulo em graus [0..180]
esp_err_t servo_write_gripper(float angle_deg);
esp_err_t servo_write_wrist(float angle_deg);

// leitura (último valor aplicado)
float servo_last_gripper(void);
float servo_last_wrist(void);

#ifdef __cplusplus
}
#endif

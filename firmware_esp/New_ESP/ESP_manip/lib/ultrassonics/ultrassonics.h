#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef US_NUM_SENSORS
#define US_NUM_SENSORS 3
#endif

#ifndef US_TRIG_PINS
#define US_TRIG_PINS   {26, 17, 19}
#endif // ESQUERDA -> Front left; FRENTE -> Front right; TRAS -> Back 

#ifndef US_ECHO_PINS
#define US_ECHO_PINS   {25, 16, 18}
#endif

#ifndef US_MAX_DISTANCE_M
#define US_MAX_DISTANCE_M 2.0f 
#endif

esp_err_t ultrassonic_init(void);

void ultrassonic_sample_once(void);

float ultrassonic_read_cm(int idx);

#ifdef __cplusplus
}
#endif

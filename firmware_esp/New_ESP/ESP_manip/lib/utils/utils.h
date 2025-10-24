#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile float STEPPER_ROTATORY_BASE;
extern volatile float STEPPER_ARM;          

extern volatile float SERVO_WRIST;        
extern volatile float SERVO_GRIPPER;     

extern volatile float   G_US_CM[3];
extern volatile uint64_t G_US_TS_MS;

#ifdef __cplusplus
}
#endif

#include "utils.h"
#include <math.h>  

volatile float   G_US_CM[3]  = {NAN, NAN, NAN};
volatile uint64_t G_US_TS_MS = 0ULL;

// Iniciais (ajuste se quiser defaults diferentes)
volatile float STEPPER_ROTATORY_BASE = 0.0f;
volatile float STEPPER_ARM           = 0.0f;

volatile float SERVO_WRIST           = 0.0f;  
volatile float SERVO_GRIPPER         = 0.0f;  


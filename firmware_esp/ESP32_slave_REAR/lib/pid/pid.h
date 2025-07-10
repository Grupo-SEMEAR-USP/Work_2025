#ifndef PID_H
#define PID_H

/**
 * @file pid.c
 * @brief PID logic implementation for motor velocity control.
*
 * Authors: Vinicius Gustierrez 
 *
 * Implements PID controllers for two motors, enabling velocity adjustments based on target values received, 
 * potentially from ROS. It dynamically adjusts motor PWM to control velocity, aiming to match the target 
 * velocity through PID feedback mechanisms.
 *
 */

/* Dependencies */
#include "pid_ctrl.h"
#include "encoder.h"
#include "h_bridge.h"
#include "utils.h"
#include "math.h"

/* PID control parameters definition. */
#define KP_L 12.5 // Proportional gain for the left motor
#define KI_L 0     // Integral gain for the left motor
#define KD_L 0   // Derivative gain for the left motor
#define TICKS_TO_RADS_LEFT 0.01570  // Conversion factor from encoder ticks to RPM for the left motor

#define KP_R 12.5    // Proportional gain for the right motor
#define KI_R 0     // Integral gain for the right motor
#define KD_R 0  // Derivative gain for the right motor
#define TICKS_TO_RADS_RIGHT 0.01570 // Conversion factor from encoder ticks to RPM for the right motor

/* Output limits to prevent excessive control signals */
#define Max_Output 1023
#define Min_Output -1023
#define Max_integral 200
#define Min_integral 0

#define PERIOD 50  // Control loop period (ms)

/* PID control sides enumeration */
typedef enum {
    PID_LEFT = 0,
    PID_RIGHT = 1
} pid_side_t;

/* Macros to select PID parameters based on motor side */
#define PID_SIDE_KP(NUM) ((NUM) == PID_LEFT ? KP_L : KP_R)
#define PID_SIDE_KI(NUM) ((NUM) == PID_LEFT ? KI_L : KI_R)
#define PID_SIDE_KD(NUM) ((NUM) == PID_LEFT ? KD_L : KD_R)

#define PID_TICKS_TO_RADS(NUM) ((NUM) == PID_LEFT ? TICKS_TO_RADS_LEFT : TICKS_TO_RADS_RIGHT)

/**
 * @brief Initializes PID controllers for each motor.
 * 
 * Sets up PID control blocks with distinct KI, KD, and KP values for each motor, 
 * enabling independent control of left and right motor velocities.
 * 
 * @param side Selects which motor's PID to initialize (left or right).
 * @return Handle to the initialized PID control block.
 */
pid_ctrl_block_handle_t init_pid(pid_side_t side);

/**
 * @brief Calculates and updates PWM values based on velocity error.
 * 
 * Determines the error between current and target velocities, then adjusts PWM outputs
 * for both motors to reach the desired velocity. The adjustment is based on PID control logic,
 * with update frequency defined by the communications setup, aiming for target velocities 
 * specified by ROS.
 * 
 * @param upcnt_unit_L Left motor's encoder configuration.
 * @param pid_block_L PID control block for the left motor.
 * @param upcnt_unit_R Right motor's encoder configuration.
 * @param pid_block_R PID control block for the right motor.
 * @return esp_err_t Error code (ESP_OK on success).
 */
esp_err_t pid_calculate(pcnt_unit_handle_t upcnt_unit_L, pid_ctrl_block_handle_t pid_block_L, pcnt_unit_handle_t upcnt_unit_R, pid_ctrl_block_handle_t pid_block_R);

#endif // PID_H

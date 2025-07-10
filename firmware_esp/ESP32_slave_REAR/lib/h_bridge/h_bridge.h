#ifndef PONTEH_H
#define PONTEH_H

/**
 * @file h_bridge.c
 * @brief Implementation of the H-bridge motor driver
 *
 * Authors: Matheus Paiva Angarola
 *
 * This file contains the implementation of the driver for an H-bridge motor controller. 
 * The driver initializes GPIO pins for motor control, configures PWM for motor speed control,
 * and provides functions to update the motor speed and direction. The H-bridge motor driver
 * can control motors connected to an H-bridge motor driver circuit, which allows bidirectional
 * control of DC motors.
 *
 *          
 */

/* Bib */
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

/* Enum */
typedef enum{
    LEFT = 0,
    RIGHT = 1
} motor_side_t;


/* Macros */

//GPIOs levels
#define LOW     0
#define HIGH    1

// GPIOs of h-bridge for motor A
#define INPUT_RIGHT_1    GPIO_NUM_4
#define INPUT_RIGHT_2    GPIO_NUM_2

// GPIOs of h-bridge for motor B
#define INPUT_LEFT_1   GPIO_NUM_27
#define INPUT_LEFT_2   GPIO_NUM_32

// PWM Config 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT   // Set duty resolution to 10 bits
#define LEDC_FREQUENCY          (5000)              // Frequency of 5kHz

#define LEDC_OUTPUT_LEFT        GPIO_NUM_26 // Enable PWM A
#define LEDC_CHANNEL_LEFT       LEDC_CHANNEL_0

#define LEDC_OUTPUT_RIGHT       GPIO_NUM_25 // Enable PWM B 
#define LEDC_CHANNEL_RIGHT      LEDC_CHANNEL_1

/* Macro functions */

#define MOTOR_INPUT_1(MOTOR) MOTOR == (LEFT) ? INPUT_LEFT_1 : INPUT_RIGHT_1
#define MOTOR_INPUT_2(MOTOR) MOTOR == (LEFT) ? INPUT_LEFT_2 : INPUT_RIGHT_2
#define MOTOR_CHANNEL(MOTOR) MOTOR == (LEFT) ? LEDC_CHANNEL_LEFT : LEDC_CHANNEL_RIGHT

/* Functions */

/**
 * @brief Initialize GPIOs
 * 
 * Initialize h-bridge pins
 * 
 * @return esp_err_t
 */
void init_gpio();

/**
 * @brief Initialize PWM configuration
 * 
 * Initialize PWM configuration of timers and channels using ledc
 * 
 * @return esp_err_t 
 */
void init_pwm();

/**
 * @brief Update motor speed and direction
 * 
 * Update motor speed and diretion by using the motor we want to change and applying action control
 * 
 * @param motor Motor side (Left or Right)
 * @param u Action control (Angular Velocity)
 * @return esp_err_t 
 */
esp_err_t update_motor(motor_side_t motor, int u);

/**
 * @brief Set h-bridge for clockwise rotation
 * 
 * Set input pins on the h-bridge to rotate the wheel clockwise
 * 
 * @param motor Motor side (Left or Right)
 * @return esp_err_t 
 */
esp_err_t _set_forward(motor_side_t motor);

/**
 * @brief Set h-bridge for counter-clockwise rotation
 * 
 * Set input pins on the h-bridge to rotate the wheel counter clockwise
 * @param motor Motor side (Left or Right)
 * @return esp_err_t 
 */
esp_err_t _set_backward(motor_side_t motor);

#endif
#ifndef SINCRONIZATION_H
#define SINCRONIZATION_H

/**
 * @file sincronization.c
 * @brief Synchronization utilities for shared access to I2C and target control variables.
 *
 * Provides thread-safe mechanisms using FreeRTOS semaphores to protect critical sections
 * related to I2C communication (read/write operations) and shared target values accessed
 * by both the I2C communication task and the PID control task. This prevents race conditions
 * and ensures data integrity across tasks running on separate cores.
 *
 * Author: [Your Name]
 */

/* Dependencies */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

/**
 * @brief Initializes synchronization semaphores for I2C and target value protection.
 *
 * This function must be called before any attempt to lock/unlock the I2C bus or access
 * the target values to ensure proper mutex initialization.
 */
void sync_init(void);

/**
 * @brief Attempts to lock the I2C mutex for thread-safe access.
 *
 * This function prevents concurrent access to the I2C peripheral.
 *
 * @param timeout Maximum time to wait for the mutex (in ticks).
 * @return true if the lock was acquired, false otherwise.
 */
bool sync_i2c_lock(TickType_t timeout);

/**
 * @brief Releases the I2C mutex after thread-safe access is complete.
 */
void sync_i2c_unlock(void);

/**
 * @brief Thread-safe setter for target values.
 *
 * Safely updates the left and right motor target velocities, used by both I2C and PID tasks.
 *
 * @param left  Target velocity for the left motor (in rad/s).
 * @param right Target velocity for the right motor (in rad/s).
 */
void sync_set_target(float left, float right);

/**
 * @brief Thread-safe getter for target values.
 *
 * Retrieves the most recent left and right motor target velocities.
 *
 * @param left  Pointer to store the left motor target value.
 * @param right Pointer to store the right motor target value.
 */
void sync_get_target(float *left, float *right);

#endif // SINCRONIZATION_H

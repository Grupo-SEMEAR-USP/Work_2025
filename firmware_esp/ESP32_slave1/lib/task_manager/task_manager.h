#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "utils.h"
#include "h_bridge.h"
#include "encoder.h"
#include "pid.h"
#include "driver/gpio.h"
#include "i2c_communication.h"

#define FREQ_COMMUNICATION 20

/**
 * @brief Task for communication and writing to the I2C bus
 *
 * This function is executed as a task and is responsible for coordinating communication
 * between reading and writing data on the I2C bus. It reads values, waits,
 * and then writes the read values to the bus.
 *
 * @param params Task parameters (not used).
 */
void i2c_task_com();

/**
 * @brief Control task
 *
 * This function is executed as a task and handles motor control and other
 * related aspects. Values read from the I2C bus are used here for control.
 *
 * @param params Task parameters (not used).
 */
void i2c_task_controle();

/**
 * @brief Creates tasks for reading and writing data on the I2C bus
 *
 * This function creates tasks for reading and writing data on the I2C bus,
 * in addition to initializing the I2C bus driver.
 *
 * @param parametros Task parameters (not used).
 * @param display Flag for display control.
 * @return
 *     - ESP_OK: Tasks created successfully.
 *     - Other error codes in case of failure.
 */
esp_err_t create_tasks();

#endif // __TASK_MANAGER_H__

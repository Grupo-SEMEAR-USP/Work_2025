#ifndef _I2CCOMMUNICATION_
#define _I2CCOMMUNICATION_

/**
 * @brief Code for I2C communication in slave mode.
 * 
 * Authors: Matheus Paiva Angarola and William Noboru Yoshihara 
 * 
 * This code sets up and starts the I2C bus in slave mode for communication
 * with a master. It reads and writes data on the I2C bus, unpacks and packs values.
 * The code creates tasks for reading and writing data and also for control functions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "utils.h"
#include "h_bridge.h"
#include "encoder.h"
#include "pid.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Definition of I2C bus parameters
#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_SCL_IO 22 // ESP32 SLC Pin
#define I2C_SLAVE_SDA_IO 21 // ESP32 SDA Pin
#define I2C_SLAVE_TX_BUF_LEN 256 
#define I2C_SLAVE_RX_BUF_LEN 256 

// Definition of values used in read and write
#define WRITE_LEN_VALUE 8
#define READ_LEN_VALUE 10
#define TIMEOUT_MS 10

// Definition of values for packing and unpacking
#define LAST_BYTE_MASK 0xFF

// Definition of frequencies used within tasks
#define FREQ_COMMUNICATION 20

extern QueueHandle_t i2c_write_queue; 
extern const char *TAG; // TAG used for console display

// Function prototypes:

/**
 * @brief Initializes the I2C bus driver in slave mode
 *
 * This function initializes the I2C driver in slave mode, configuring SDA and SCL pins,
 * setting up the I2C bus configuration, and installing the driver.
 *
 * @return
 *     - ESP_OK: I2C driver initialized successfully.
 *     - ESP_ERR_INVALID_ARG: Initialization failed due to invalid arguments.
 *     - ESP_ERR_NO_MEM: Initialization failed due to insufficient available memory.
 *     - ESP_FAIL: Other errors in I2C driver initialization.
 */
esp_err_t i2c_slave_init(void);

/**
 * @brief Task for reading data from the I2C bus
 *
 * This function is executed as a task and reads data from the I2C bus,
 * unpacks the read values, and returns them. Values are sent by another task.
 *
 * @param params Task parameters (not used).
 */
void i2c_read_task();

/**
 * @brief Task for writing data to the I2C bus
 *
 * This function is executed as a task and sends the data received by another task
 * to the I2C bus. Values are packed before sending.
 *
 * @param params Task parameters (not used).
 * @param value The value to be written to the I2C bus.
 */
void i2c_write_task(int value_r, int value_l) ;

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

/**
 * @brief Resets the I2C bus
 *
 * This function resets the I2C bus, which includes reinitializing the bus
 * to a known state. It ensures that any errors or issues in the bus
 * communication are cleared, preparing it for future operations.
 *
 * @param i2c_num The I2C port number to reset.
 *        - Use `I2C_NUM_0` for the first I2C bus.
 *        - Use `I2C_NUM_1` for the second I2C bus.
 * @return
 *     - ESP_OK: I2C bus reset successfully.
 *     - ESP_ERR_INVALID_ARG: Invalid I2C port number.
 *     - Other error codes in case of failure.
 */
 esp_err_t reset_i2c(i2c_port_t i2c_num);

 /**
 * @brief Displays the contents of a buffer
 *
 * This function displays the contents of the given buffer in a readable
 * format. It iterates through the buffer and prints each byte in hexadecimal
 * format, which is useful for debugging or visualizing the data stored in the buffer.
 *
 * @param buf Pointer to the buffer containing the data to display.
 * @param len The length of the buffer to display.
 * @return
 *     - None
 */
 void disp_buf(uint8_t *buf, int len);


#endif

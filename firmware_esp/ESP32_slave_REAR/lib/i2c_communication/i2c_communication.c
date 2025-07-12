#include "i2c_communication.h"
#include "utils.h"

QueueHandle_t i2c_write_queue = NULL; 
const char *TAG = "i2c-teste"; 

esp_err_t i2c_slave_init(void) {
    
    int i2c_slave_port = I2C_SLAVE_NUM;

    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,         
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // Internal SDA PULLUP disable
        .scl_io_num = I2C_SLAVE_SCL_IO,          
        .scl_pullup_en = GPIO_PULLUP_DISABLE, // Internal SCL PULLUP disable
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.maximum_speed = 400000,
        .slave.slave_addr = I2C_SLAVE_ADDRESS,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);

    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void disp_buf(uint8_t *buf, int len) {
    int i;

    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

void i2c_read_task() {

    uint8_t rx_data[I2C_SLAVE_RX_BUF_LEN];

    if (sync_i2c_lock(pdMS_TO_TICKS(10))) {

        // Leia tudo que tiver no buffer de uma vez
        int size = i2c_slave_read_buffer(
            I2C_SLAVE_NUM,
            rx_data,
            I2C_SLAVE_RX_BUF_LEN,
            TIMEOUT_MS / portTICK_PERIOD_MS
        );

        if (size > 0) {
            if (size < 9) {
                sync_i2c_unlock();
                return;
            }

            uint8_t payload[9];
            memcpy(payload, rx_data + 1, 9);

            int32_t read_value_r = 0;
            int32_t read_value_l = 0;

            read_value_r = (payload[1] << 24) |
                           (payload[2] << 16) |
                           (payload[3] << 8) |
                           (payload[4]);

            read_value_l = (payload[5] << 24) |
                           (payload[6] << 16) |
                           (payload[7] << 8) |
                           (payload[8]);

            // ESP_LOGI(TAG, "Read values: R=%ld, L=%ld", (long)read_value_r, (long)read_value_l);

            if (read_value_r > MAX_ACCEPTABLE_VALUE ||
                read_value_l > MAX_ACCEPTABLE_VALUE) {
                // ESP_LOGW(TAG, "Values out of range: L=%ld, R=%ld", (long)read_value_l, (long)read_value_r);
                sync_i2c_unlock();
                return;
            }

            TARGET_VALUE_R = read_value_r;
            TARGET_VALUE_L = read_value_l;
    
            TARGET_VALUE_R = TARGET_VALUE_R / 1000;
            TARGET_VALUE_L = TARGET_VALUE_L / 1000;
    
            fail_count = 0;

        } else {
            // ESP_LOGI(TAG, "Read failed! Restarting I2C...");
            fail_count++;
            reset_i2c(I2C_SLAVE_NUM);
            if (fail_count >= 3) esp_restart();
        }

        sync_i2c_unlock();
    } else {
        // ESP_LOGW(TAG, "I2C busy, skip read.");
    }
}

void i2c_write_task(pcnt_unit_handle_t upcnt_unit_R, pcnt_unit_handle_t upcnt_unit_L) {

    static int64_t last_time_us = 0;

    int64_t now_us = esp_timer_get_time();
    
    int64_t delta_us = 0;
    if (last_time_us != 0) {
        delta_us = (now_us - last_time_us);
    }
    last_time_us = now_us;

    ENCODER_READ_L = pulse_count(upcnt_unit_L);
    ENCODER_READ_R = pulse_count(upcnt_unit_R);

    RADS_L = (ENCODER_READ_L * 2 * PI / (ENCODER_RESOLUTION * delta_us)) * 1000000;
    RADS_R = (ENCODER_READ_R * 2 * PI / (ENCODER_RESOLUTION * delta_us)) * 1000000;

    int32_t left = RADS_L * 1000;
    int32_t right = RADS_R * 1000;


    // Payload: 9 bytes (1 byte header + 4 bytes R + 4 bytes L)
    uint8_t tx_data[9];
    tx_data[0] = 0x01;

    // Empacota value_r (4 bytes)
    tx_data[1] = (right >> 24) & 0xFF;
    tx_data[2] = (right >> 16) & 0xFF;
    tx_data[3] = (right >> 8) & 0xFF;
    tx_data[4] = right & 0xFF;

    // Empacota value_l (4 bytes)
    tx_data[5] = (left >> 24) & 0xFF;
    tx_data[6] = (left >> 16) & 0xFF;
    tx_data[7] = (left >> 8) & 0xFF;
    tx_data[8] = left & 0xFF;

    if (sync_i2c_lock(pdMS_TO_TICKS(10))) {

        int size = i2c_slave_write_buffer(
            I2C_SLAVE_NUM,
            tx_data,
            9,
            TIMEOUT_MS / portTICK_PERIOD_MS
        );

        sync_i2c_unlock();

        // ESP_LOG_BUFFER_HEX("I2C TX", tx_data, 9);

        // Log opcional
        /*
        if (size > 0) {
            ESP_LOGI(TAG, "I2C Write OK: R=%d L=%d", value_r / 1000, value_l / 1000);
        } else {
            ESP_LOGW(TAG, "I2C write failed!");
        }
        */
    } else {
        // ESP_LOGW(TAG, "I2C busy, skipping write.");
    }
}


void i2c_init() {
    ESP_ERROR_CHECK(i2c_slave_init());  // I2C initial restart
    i2c_write_queue = xQueueCreate(10, I2C_SLAVE_TX_BUF_LEN);
}

esp_err_t reset_i2c(i2c_port_t i2c_num) {
    esp_err_t ret = i2c_driver_delete(i2c_num);

    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return i2c_slave_init();
}

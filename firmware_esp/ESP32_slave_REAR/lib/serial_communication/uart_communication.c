// uart_comm.c

#include "uart_communication.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_COMM";

void uart_comm_read(void)
{
    uint8_t rx_data[128];

    int size = fread(rx_data, 1, 9, stdin);

    if (size > 0)
    {
        if (size < 9)
        {
            return;
        }

        uint8_t header = rx_data[0];
        if (header != 0x01)
        {
            return;
        }

        int32_t value_r = (rx_data[1] << 24) |
                          (rx_data[2] << 16) |
                          (rx_data[3] << 8) |
                          (rx_data[4]);

        int32_t value_l = (rx_data[5] << 24) |
                          (rx_data[6] << 16) |
                          (rx_data[7] << 8) |
                          (rx_data[8]);


        if (value_r > MAX_ACCEPTABLE_VALUE || value_l > MAX_ACCEPTABLE_VALUE)
        {
            return;
        }

        // ESP_LOGI(TAG, "Recebido: R=%ld L=%ld", (long)value_r, (long)value_l);

        TARGET_VALUE_R = value_r;
        TARGET_VALUE_L = value_l;

        TARGET_VALUE_R = TARGET_VALUE_R / 1000;
        TARGET_VALUE_L = TARGET_VALUE_L / 1000;
    }
}

void uart_comm_write(pcnt_unit_handle_t upcnt_unit_R, pcnt_unit_handle_t upcnt_unit_L)
{
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

    // ESP_LOGI(TAG, "Enviado: R=%ld L=%ld", (long)value_r, (long)value_l);

    uint8_t tx_data[9];
    tx_data[0] = 0x02; 

    tx_data[1] = (left >> 24) & 0xFF;
    tx_data[2] = (left >> 16) & 0xFF;
    tx_data[3] = (left >> 8) & 0xFF;
    tx_data[4] = left & 0xFF;

    tx_data[5] = (right >> 24) & 0xFF;
    tx_data[6] = (right >> 16) & 0xFF;
    tx_data[7] = (right >> 8) & 0xFF;
    tx_data[8] = right & 0xFF;

    fwrite(tx_data, 1, 9, stdout);
    fflush(stdout); 
}

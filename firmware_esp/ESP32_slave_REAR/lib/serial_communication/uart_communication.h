#ifndef _UARTCOMMUNICATION_
#define _UARTCOMMUNICATION_

#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "esp_timer.h"
#include "encoder.h"

/**
 * @brief Lê dados da UART.
 * 
 * @param buffer Buffer de destino para os dados.
 * @param max_length Tamanho máximo do buffer.
 * @param timeout_ms Timeout em milissegundos.
 * @return Número de bytes lidos.
 */
void uart_comm_read();
void uart_comm_write(pcnt_unit_handle_t upcnt_unit_R, pcnt_unit_handle_t upcnt_unit_L);


#endif
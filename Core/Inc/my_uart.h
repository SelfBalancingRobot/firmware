/*
 * my_uart.h
 *
 *  Created on: Feb 22, 2026
 *      Author: ozker
 */

#ifndef INC_MY_UART_H_
#define INC_MY_UART_H_

#include "stdbool.h"
#include "stdint.h"
#include "usart.h"

#define MAX_MESSAGE_LENGTH 80

typedef struct{
    UART_HandleTypeDef *uart_handle;
    char message_buffer[MAX_MESSAGE_LENGTH];
    volatile uint8_t message_length;
    volatile bool message_ready;
    uint8_t rx_byte;
}uart_t;

bool my_uart_init(uart_t *uart);
bool my_uart_get_message(uart_t *uart, char* message);
void my_uart_printf(uart_t *uart, const char *format, ...);
#endif /* INC_MY_UART_H_ */

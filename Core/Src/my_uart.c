/*
 * my_uart.c
 *
 *  Created on: Feb 22, 2026
 *      Author: ozker
 */
#include "my_uart.h"
#include "stdarg.h"
#include "stdio.h"

#define UARTS_MAX (sizeof(uarts) / sizeof(uarts[0]))

static uint8_t uart_idx = 0;
static uart_t *uarts[12] = {0};

// rozpoczecie odbioru danych
static void my_uart_start_receiving(uart_t *uart){
    HAL_UART_Receive_IT(uart->uart_handle, &uart->rx_byte, 1);
}

bool my_uart_init(uart_t *uart){
    if(!uart || !uart->uart_handle){
        return false;
    }if(uart_idx > UARTS_MAX){
        return false;
    }
    uarts[uart_idx] = uart;
    uart->message_length = 0;
    uart->message_ready  = false;
    my_uart_start_receiving(uart);
    uart_idx++; 
    return true;
}

// Obsluga przerwania. Jesli przyjdzie jakas literka to zostaje ona przypisana w kolejne miejsce bufora. Jesli przyjdzie znak konca linii
//to zostaje ustawiona flaga pozwalajaca na przepisanie zawartosci bufora do jakiejs tablicy
static void my_uart_receive_procedure(uart_t *uart){
    if(uart->rx_byte == '\n' || uart->rx_byte == '\r'){
        if(uart->message_length > 0){
            uart->message_buffer[uart->message_length] = '\0';
            uart->message_ready = true;
        }
    }else if(!uart->message_ready){
        if(uart->message_length >= (MAX_MESSAGE_LENGTH - 1)){
            uart->message_length = 0;
        }
        uart->message_buffer[uart->message_length] = uart->rx_byte;
        uart->message_length++;
        
    }
    HAL_UART_Receive_IT(uart->uart_handle, &uart->rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    for(uint8_t i = 0; i < uart_idx; i++){
        if(uarts[i]->uart_handle == huart){
            my_uart_receive_procedure(uarts[i]);
            break;
        }
    }
}



// po ustawieniu flagi opisanej powyżej zostaje skopiowany bufor do tablicy podanej jako argument tej funkcji
bool my_uart_get_message(uart_t *uart, char *message){
    if(uart->message_ready){
        for(uint8_t pos = 0; pos <= uart->message_length; pos ++){
            message[pos] = uart->message_buffer[pos];
        }
        uart->message_length = 0;
        uart->message_ready = false;
        return true;
    }else{
        return false;
    }
}

void my_uart_printf(uart_t *uart, const char *format, ...){
    char buffer[256] = {0};
    va_list args;

    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    for(int i = 0; i < len; i++){
        if(buffer[i] == '\n'){
            uint8_t cr = '\r';
            HAL_UART_Transmit(uart->uart_handle, &cr, 1, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(uart->uart_handle, (uint8_t*)&buffer[i], 1, HAL_MAX_DELAY);
    }
}

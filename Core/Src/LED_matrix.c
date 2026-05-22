/*
 * MAX7219.c
 *
 *  Created on: Mar 26, 2026
 *      Author: ozker
 */

#include "LED_matrix.h"


#define MAX7219_DELAY 100

#define MAX7219_DIGIT0_REG 0x1
#define MAX7219_DECODE_REG 0x9
#define MAX7219_INTENSITY_REG 0xA
#define MAX7219_SCAN_LIMIT_REG 0xB
#define MAX7219_SHUTDOWN_REG 0xC
#define MAX7219_DISPLAY_TEST_REG 0xF

#define MAX7219_SHUTDOWN 0x0
#define MAX7219_NO_DECODE 0x0
#define MAX7219_NO_SHUTDOWN 0x1
#define MAX7219_DISPLAY_ALL_DIGITS 0x7
#define MAX7219_NO_TEST 0x0

static void max7219_write(LED_matrix_t *matrix, uint8_t reg, uint8_t val){
	uint8_t data[2] = {reg, val};
	HAL_GPIO_WritePin(matrix->cs_port, matrix->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(matrix->spi_handle, data, 2, MAX7219_DELAY);
	HAL_GPIO_WritePin(matrix->cs_port, matrix->cs_pin, GPIO_PIN_SET);
}

void LED_matrix_init(LED_matrix_t *matrix){
	max7219_write(matrix, MAX7219_SHUTDOWN_REG, MAX7219_SHUTDOWN);
	max7219_write(matrix, MAX7219_DECODE_REG, MAX7219_NO_DECODE);
	max7219_write(matrix, MAX7219_INTENSITY_REG, 0xA); // 0x0 - no light, 0xF - max brightness
	max7219_write(matrix, MAX7219_SCAN_LIMIT_REG, MAX7219_DISPLAY_ALL_DIGITS);
	max7219_write(matrix, MAX7219_DISPLAY_TEST_REG, MAX7219_NO_TEST);
	
	for(uint8_t row = MAX7219_DIGIT0_REG; row < MAX7219_DIGIT0_REG + 8; row++){
		max7219_write(matrix, row, 0x00);
	}
	LED_matrix_clear_buffer(matrix);

	max7219_write(matrix, MAX7219_SHUTDOWN_REG, MAX7219_NO_SHUTDOWN);
}

void LED_matrix_set_pixel(LED_matrix_t *matrix, uint8_t x, uint8_t y){
	if(x > 7){
		x = 7;
	}
	if(y > 7){
		y = 7;
	}
	matrix->buffer[y] |= 1 << x;
}

void LED_matrix_reset_pixel(LED_matrix_t *matrix, uint8_t x, uint8_t y){
	if(x > 7){
		x = 7;
	}
	if(y > 7){
		y = 7;
	}
	matrix->buffer[y] &= ~(1 << x);
}

void LED_matrix_clear_buffer(LED_matrix_t *matrix){
	for(uint8_t row = 0; row < 8; row++){
		matrix->buffer[row] = 0;
	}
}

void LED_matrix_load_buffer(LED_matrix_t *matrix, uint8_t *buffer){
	for(uint8_t row = 0; row < 8; row++){
		matrix->buffer[row] = buffer[row];
	}
}

void LED_matrix_draw_buffer(LED_matrix_t *matrix){
    for(uint8_t row = 0; row < 8; row++){
    	max7219_write(matrix, MAX7219_DIGIT0_REG + row, matrix->buffer[row]);
    }
}

/*
 * MAX7219.h
 *
 *  Created on: Mar 26, 2026
 *      Author: ozker
 */

#ifndef INC_LED_MATRIX_H_
#define INC_LED_MATRIX_H_

#include "spi.h"
#include "gpio.h"
#include "stdbool.h"
typedef struct{
	SPI_HandleTypeDef *spi_handle;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	uint8_t buffer[8];
}LED_matrix_t;

typedef enum{
	ROTATE_CW,
	ROTATE_CCW
}rotation_t;

void LED_matrix_init(LED_matrix_t *matrix);
void LED_matrix_set_pixel(LED_matrix_t *matrix, uint8_t x, uint8_t y);
void LED_matrix_reset_pixel(LED_matrix_t *matrix, uint8_t x, uint8_t y);
void LED_matrix_clear_buffer(LED_matrix_t *matrix);
void LED_matrix_load_buffer(LED_matrix_t *matrix, uint8_t *buffer);
void LED_matrix_rotate_buffer(LED_matrix_t *matrix, rotation_t direction);
void LED_matrix_draw_buffer(LED_matrix_t *matrix);
#endif /* INC_LED_MATRIX_H_ */

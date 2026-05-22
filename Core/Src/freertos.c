/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dc_motor.h"
#include "my_uart.h"
#include "MPU6050.h"
#include "LED_matrix.h"
#include "balance_regulator.h"
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
motor_t motor1 = {
  .pwm_tim = &htim3,
  .pwm_channel_A = TIM_CHANNEL_1,
  .pwm_channel_B = TIM_CHANNEL_2,
  .encoder_tim = &htim1,
  .encoder_curr_count = 0,
  .encoder_prev_count = 0,
  .encoder_resolution = 700,
  .measured_rpm = 0,
  .max_rpm = 251.0f,
  .regulator_Kp = 1.5f,
  .regulator_Ki = 0.05f,
  .integral = 0,
  .sign = -1

};
motor_t motor2 = {
  .pwm_tim = &htim3,
  .pwm_channel_A = TIM_CHANNEL_3,
  .pwm_channel_B = TIM_CHANNEL_4,
  .encoder_tim = &htim2,
  .encoder_curr_count = 0,
  .encoder_prev_count = 0,
  .encoder_resolution = 700,
  .measured_rpm = 0,
  .max_rpm = 251.0f,
  .regulator_Kp = 1.5f,
  .regulator_Ki = 0.2f,
  .integral = 0,
  .sign = 1
};
LED_matrix_t ledMatrix1 = {
  .spi_handle = &hspi1,
  .cs_port = matrix1_cs_GPIO_Port,
  .cs_pin = matrix1_cs_Pin
};
LED_matrix_t ledMatrix2 = {
  .spi_handle = &hspi1,
  .cs_port = matrix2_cs_GPIO_Port,
  .cs_pin = matrix2_cs_Pin
};
uart_t bluetooth = {
  .uart_handle = &huart2
};
balance_pid_t balance_pid = {
	.target_angle = 0.0f,
	.Kp = 20.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.integral = 0.0f,
	.prev_error = 0.0f
};

float current_angle = 0.0f;

/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for BluetoothTask */
osThreadId_t BluetoothTaskHandle;
const osThreadAttr_t BluetoothTask_attributes = {
  .name = "BluetoothTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LedMatrixTask */
osThreadId_t LedMatrixTaskHandle;
const osThreadAttr_t LedMatrixTask_attributes = {
  .name = "LedMatrixTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartInitSemaphore */
osSemaphoreId_t UartInitSemaphoreHandle;
const osSemaphoreAttr_t UartInitSemaphore_attributes = {
  .name = "UartInitSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartBluetoothTask(void *argument);
void StartLedMatrixTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UartInitSemaphore */
  UartInitSemaphoreHandle = osSemaphoreNew(1, 1, &UartInitSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of BluetoothTask */
  BluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &BluetoothTask_attributes);

  /* creation of LedMatrixTask */
  LedMatrixTaskHandle = osThreadNew(StartLedMatrixTask, NULL, &LedMatrixTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  osSemaphoreAcquire(UartInitSemaphoreHandle, 0);
  uint32_t Ts_ms = 10;
  float Ts_s = (float)Ts_ms / 1000.0f;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  motor_init(&motor2);
  motor_init(&motor1);
  osDelay(100);
  mpu_init(&hi2c1);


  mpu_count_accel_offset();
  mpu_count_gyro_offset();
  osSemaphoreRelease(UartInitSemaphoreHandle);

  /* Infinite loop */
  for(;;)
  {
	mpu_get_accel(&ax, &ay, &az);
	mpu_get_gyro(&gx, &gy, &gz);
	current_angle = mpu_count_angle(ay, az, gx, Ts_s);
    if(current_angle > 45.0f || current_angle < -45.0f) {
        motor_set_rpm(&motor1, 0, Ts_s);
        motor_set_rpm(&motor2, 0, Ts_s);
        balance_pid.integral = 0; // zeruj całkę
        motor1.integral = 0;
        motor2.integral = 0;
    }else{
    	float target_rpm = balance_count_target_rpm(&balance_pid, current_angle, Ts_s);
    	motor_set_rpm(&motor2, target_rpm, Ts_s);
    	motor_set_rpm(&motor1, target_rpm, Ts_s);
    }

    osDelay(Ts_ms);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  char received_message[30] = {""};
  uint16_t p = 0;
  uint16_t i = 0;
  uint16_t d = 0;
  my_uart_init(&bluetooth);
  osSemaphoreAcquire(UartInitSemaphoreHandle, osWaitForever);
  /* Infinite loop */
  for(;;)
  {

	  my_uart_printf(&bluetooth, "motor1 rpm = %.2f\n", motor1.measured_rpm);
	  my_uart_printf(&bluetooth, "motor2 rpm = %.2f\n", motor2.measured_rpm);
	  my_uart_printf(&bluetooth, "current angle = %.2f\n", current_angle);

	  if(my_uart_get_message(&bluetooth, received_message) == true){
		  for(uint8_t idx = 0; idx < sizeof(received_message); idx++){
			  if(received_message[idx] == 'p'){
				  if(isdigit(received_message[idx+1]) && isdigit(received_message[idx+2]) && isdigit(received_message[idx+3])){
					  p = (received_message[idx + 1] - '0') * 100 +
						  (received_message[idx + 2] - '0') * 10 +
						  received_message[idx + 3] - '0';
					  balance_pid.Kp = p / 10 + (float)(p % 10)/ 10;
				  }

			  }
			  if(received_message[idx] == 'i'){
				  if(isdigit(received_message[idx+1]) && isdigit(received_message[idx+2]) && isdigit(received_message[idx+3])){
					  i = (received_message[idx + 1] - '0') * 100 +
						  (received_message[idx + 2] - '0') * 10 +
						  received_message[idx + 3] - '0';
					  balance_pid.Ki = i / 100 + (float)(i % 100)/ 100;
				  }

			  }
			  if(received_message[idx] == 'd'){
				  if(isdigit(received_message[idx+1]) && isdigit(received_message[idx+2]) && isdigit(received_message[idx+3])){
					  d = (received_message[idx + 1] - '0') * 100 +
						  (received_message[idx + 2] - '0') * 10 +
						  received_message[idx + 3] - '0';
					  balance_pid.Kd = d / 100 + (float)(d % 100)/ 100;
				  }

			  }
			  if(received_message[idx] == 'P'){
				  if(isdigit(received_message[idx+1]) && isdigit(received_message[idx+2]) && isdigit(received_message[idx+3])){
					  p = (received_message[idx + 1] - '0') * 100 +
						  (received_message[idx + 2] - '0') * 10 +
						  received_message[idx + 3] - '0';
					  float P = p / 100 + (float)(p % 100)/ 100;
					  motor1.regulator_Kp = P;
					  motor2.regulator_Kp = P;
				  }

			  }
			  if(received_message[idx] == 'I'){
				  if(isdigit(received_message[idx+1]) && isdigit(received_message[idx+2]) && isdigit(received_message[idx+3])){
					  i = (received_message[idx + 1] - '0') * 100 +
						  (received_message[idx + 2] - '0') * 10 +
						  received_message[idx + 3] - '0';
					  float I = i / 100 + (float)(i % 100)/ 100;
					  motor1.regulator_Ki = I;
					  motor2.regulator_Ki = I;
				  }

			  }
		  }
	  }
	  osDelay(500);
  }
  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartLedMatrixTask */
/**
* @brief Function implementing the LedMatrixTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedMatrixTask */
void StartLedMatrixTask(void *argument)
{
  /* USER CODE BEGIN StartLedMatrixTask */
  bool sixSeven = true;
  uint8_t pattern6[8] = {
	0b00111100,
	0b01100110,
	0b11000000,
	0b11111100,
	0b11000110,
	0b11000110,
	0b01100110,
	0b00111100
  };

  uint8_t pattern7[8] = {
	0b11111110,
	0b00000110,
	0b00001100,
	0b00011000,
	0b00110000,
	0b00110000,
	0b00110000,
	0b00110000
  };

  LED_matrix_init(&ledMatrix1);
  osDelay(10);
  LED_matrix_init(&ledMatrix2);
  /* Infinite loop */
  for(;;)
  {
	LED_matrix_clear_buffer(&ledMatrix1);
	LED_matrix_clear_buffer(&ledMatrix2);
	if(sixSeven){

	  LED_matrix_load_buffer(&ledMatrix1, pattern6);
	  LED_matrix_load_buffer(&ledMatrix2, pattern7);
	  sixSeven = false;
	}
	else{
	  LED_matrix_load_buffer(&ledMatrix1, pattern7);
	  LED_matrix_load_buffer(&ledMatrix2, pattern6);
	  sixSeven = true;
	}

	LED_matrix_draw_buffer(&ledMatrix1);
	LED_matrix_draw_buffer(&ledMatrix2);
	osDelay(1000);
  }
  /* USER CODE END StartLedMatrixTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


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
#include "command_parser.h"
#include "velocity_regulator.h"
#include "angle_regulator.h"
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
  .ramped_rpm = 0,
  .ramp_step = 8,
  .max_rpm = 251.0f,
  .regulator_Kp = 0.8f,
  .regulator_Ki = 0.0f,
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
  .ramped_rpm = 0,
  .ramp_step = 8,
  .max_rpm = 251.0f,
  .regulator_Kp = 0.8f,
  .regulator_Ki = 0.0f,
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
	.Kp = 10.0f,
	.Ki = 0.0f,
	.Kd = 5.0f,
	.integral = 0.0f,
	.prev_error = 0.0f,
	.rpm_death_band = 10.0f

};

mpu_kalman_t mpu_kalman = {
	.Q_angle = 0.001f,
	.Q_bias = 0.003f,
	.R_measure = 0.03f,
	.angle = 0.0f,
	.bias = 0.0f,
	.P = {{0.0f, 0.0f},{0.0f, 0.0f}},
	.initialized = false
};

angle_pid_t angle_pid = {
	.Kp = 1.0f,
	.Ki = 0.0f,
	.Kd = 0.5f,
	.integral = 0.0f,
	.prev_error = 0.0f

};

velocity_regulator_t velocity_pid = {
	.Kp = 3.0f,
	.Ki = 0.5f,
	.Kd = 0.0f,
	.integral = 0.0f,
	.prev_error = 0.0f

};

robot_mode_t robot_mode = BALANCE;
float callibration_rpm = 0;
bool send_imu_offsets_flag = false;
bool send_pid_flag = true;
float pwm = 0.0f;
float angle_ref = 0.0f;
float calib_ang = 0.0f;
float robot_speed = 0.0f;
uint8_t outer_divider_ref = 10;
command_context_t command_context = {
	.motor1 = &motor1,
	.motor2 = &motor2,
	.ang_pid = &angle_pid,
	.vel_pid = &velocity_pid,
	.mode = &robot_mode,
	.out_div_ref = &outer_divider_ref,
	.kalman = &mpu_kalman,
	.send_imu_offsets_flag = &send_imu_offsets_flag,
	.send_pid_flag = &send_pid_flag,
	.angle = &calib_ang
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
/* Definitions for SendLogsTask */
osThreadId_t SendLogsTaskHandle;
const osThreadAttr_t SendLogsTask_attributes = {
  .name = "SendLogsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for LedMatrixTask */
osThreadId_t LedMatrixTaskHandle;
const osThreadAttr_t LedMatrixTask_attributes = {
  .name = "LedMatrixTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ReceiveCmdTask */
osThreadId_t ReceiveCmdTaskHandle;
const osThreadAttr_t ReceiveCmdTask_attributes = {
  .name = "ReceiveCmdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for HardwareInitSemaphore */
osSemaphoreId_t HardwareInitSemaphoreHandle;
const osSemaphoreAttr_t HardwareInitSemaphore_attributes = {
  .name = "HardwareInitSemaphore"
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
void StartSendLogsTask(void *argument);
void StartLedMatrixTask(void *argument);
void StartReceiveCmdTask(void *argument);

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
  /* creation of HardwareInitSemaphore */
  HardwareInitSemaphoreHandle = osSemaphoreNew(1, 1, &HardwareInitSemaphore_attributes);

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

  /* creation of SendLogsTask */
  SendLogsTaskHandle = osThreadNew(StartSendLogsTask, NULL, &SendLogsTask_attributes);

  /* creation of LedMatrixTask */
  LedMatrixTaskHandle = osThreadNew(StartLedMatrixTask, NULL, &LedMatrixTask_attributes);

  /* creation of ReceiveCmdTask */
  ReceiveCmdTaskHandle = osThreadNew(StartReceiveCmdTask, NULL, &ReceiveCmdTask_attributes);

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
  osSemaphoreAcquire(HardwareInitSemaphoreHandle, 0);
  uint32_t Ts_ms = 4;
  uint8_t outer_divider = 10;

  uint32_t nextWake = osKernelGetTickCount();
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
  send_imu_offsets_flag = true;
  osSemaphoreRelease(HardwareInitSemaphoreHandle);

  /* Infinite loop */
  for(;;)
  {
	nextWake += Ts_ms;
	mpu_get_accel(&ax, &ay, &az);
	mpu_get_gyro(&gx, &gy, &gz);
	current_angle = mpu_count_angle_Kalman(&mpu_kalman, ay, az, gx, Ts_s);
	motor_measure_rpm(&motor1, Ts_s);
	motor_measure_rpm(&motor2, Ts_s);
	float rpm_avg = 0.5f * (motor1.measured_rpm + motor2.measured_rpm);
	float wheel_omega = rpm_avg * 2.0f * 3.14f / 60.0f;
	robot_speed = wheel_omega * 0.1f;
	if(robot_mode == BALANCE){
	    if(current_angle > 45.0f || current_angle < -45.0f) {
	    	motor_set_signed_pwm(&motor1, 0.0f);
	    	motor_set_signed_pwm(&motor2, 0.0f);
	        angle_pid.integral = 0.0f;
	        velocity_pid.integral = 0.0f;
	        angle_pid.prev_error = 0.0f;
	        velocity_pid.prev_error = 0.0f;
	    }else{
	    	float v_ref = 0.0f;
	    	if (outer_divider > outer_divider_ref){
	    		angle_ref = calib_ang + velocity_pid_count_angle(&velocity_pid, v_ref, robot_speed, Ts_s * (outer_divider_ref + 1));
	    		outer_divider = 0;
	    	}else{
	    		outer_divider++;
	    	}

	    	pwm = angle_pid_count_pwm(&angle_pid, gx,-angle_ref, current_angle, Ts_s);
	    	motor_set_signed_pwm(&motor1, pwm);
	    	motor_set_signed_pwm(&motor2, pwm);
	    }
	}else if(robot_mode == CALIBRATION){
    	pwm = angle_pid_count_pwm(&angle_pid, gx, calib_ang, current_angle, Ts_s);
    	motor_set_signed_pwm(&motor1, pwm);
    	motor_set_signed_pwm(&motor2, pwm);
	}

    osDelayUntil(nextWake);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartSendLogsTask */
/**
* @brief Function implementing the SendLogsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendLogsTask */
void StartSendLogsTask(void *argument)
{
  /* USER CODE BEGIN StartSendLogsTask */
  /* Infinite loop */
  osSemaphoreAcquire(HardwareInitSemaphoreHandle, osWaitForever);
  osSemaphoreAcquire(UartInitSemaphoreHandle, osWaitForever);
  for(;;)
  {
	my_uart_printf(&bluetooth, "pwm%.2f\n", pwm);
	my_uart_printf(&bluetooth, "velcur%.2f\n", robot_speed);
	my_uart_printf(&bluetooth, "angcur%.2f\n", current_angle);
	if(robot_mode == BALANCE){
		my_uart_printf(&bluetooth, "angref%.2f\n", angle_ref);
	}
	else if(robot_mode == CALIBRATION){
		my_uart_printf(&bluetooth, "angref%.2f\n", calib_ang);
	}

	if(send_pid_flag == true){
		my_uart_printf(&bluetooth, "angpidp%.2f\n", angle_pid.Kp);
		my_uart_printf(&bluetooth, "angpidi%.2f\n", angle_pid.Ki);
		my_uart_printf(&bluetooth, "angpidd%.2f\n", angle_pid.Kd);
		my_uart_printf(&bluetooth, "velpidp%.2f\n", velocity_pid.Kp);
		my_uart_printf(&bluetooth, "velpidi%.2f\n", velocity_pid.Ki);
		my_uart_printf(&bluetooth, "velpidd%.2f\n", velocity_pid.Kd);
		my_uart_printf(&bluetooth, "kalrm%.5f\n", mpu_kalman.R_measure);
		my_uart_printf(&bluetooth, "kalqa%.5f\n", mpu_kalman.Q_angle);
		my_uart_printf(&bluetooth, "kalqb%.5f\n", mpu_kalman.Q_bias);
		send_pid_flag = false;
	}
	if(send_imu_offsets_flag == true){
    	float ax, ay, az, gx, gy, gz;
    	mpu_get_accel_offset(&ax, &ay, &az);
    	mpu_get_gyro_offset(&gx, &gy, &gz);
    	my_uart_printf(&bluetooth, "accoffx%.2f\n", ax);
    	my_uart_printf(&bluetooth, "accoffy%.2f\n", ay);
    	my_uart_printf(&bluetooth, "accoffz%.2f\n", az);
    	my_uart_printf(&bluetooth, "gyroffx%.2f\n", gx);
    	my_uart_printf(&bluetooth, "gyroffy%.2f\n", gy);
    	my_uart_printf(&bluetooth, "gyroffz%.2f\n", gz);
    	send_imu_offsets_flag = false;
    }
	osDelay(100);
  }
  /* USER CODE END StartSendLogsTask */
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
  uint8_t scary_eye[8] = {
	0b00011000,
	0b00011000,
	0b00111100,
	0b00111100,
	0b00111100,
	0b00111100,
	0b00011000,
	0b00011000
  };

  uint8_t deafult_eye[8] = {
	0b00000000,
	0b00111100,
	0b01111110,
	0b11111111,
	0b11111111,
	0b01111110,
	0b00111100,
	0b00000000
  };

  uint8_t dead_eye[8] = {
	0b11000011,
	0b11100111,
	0b01111110,
	0b00111100,
	0b00111100,
	0b01111110,
	0b11100111,
	0b11000011
  };

  LED_matrix_init(&ledMatrix1);
  osDelay(10);
  LED_matrix_init(&ledMatrix2);
  /* Infinite loop */
  for(;;)
  {
	LED_matrix_clear_buffer(&ledMatrix1);
	LED_matrix_clear_buffer(&ledMatrix2);
	if(current_angle > 45.0f || current_angle < -45.0f){
		LED_matrix_load_buffer(&ledMatrix1, dead_eye);
		LED_matrix_load_buffer(&ledMatrix2, dead_eye);
	}else if(current_angle > 7.0f || current_angle < -7.0f){
		LED_matrix_load_buffer(&ledMatrix1, scary_eye);
		LED_matrix_load_buffer(&ledMatrix2, scary_eye);
	}else{
		LED_matrix_load_buffer(&ledMatrix1, deafult_eye);
		LED_matrix_load_buffer(&ledMatrix2, deafult_eye);
	}
	LED_matrix_rotate_buffer(&ledMatrix1, ROTATE_CW);
	LED_matrix_rotate_buffer(&ledMatrix2, ROTATE_CW);
	LED_matrix_draw_buffer(&ledMatrix1);
	LED_matrix_draw_buffer(&ledMatrix2);

	osDelay(200);
  }
  /* USER CODE END StartLedMatrixTask */
}

/* USER CODE BEGIN Header_StartReceiveCmdTask */
/**
* @brief Function implementing the ReceiveCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveCmdTask */
void StartReceiveCmdTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveCmdTask */
  /* Infinite loop */
  char received_message[30] = {""};
  osSemaphoreAcquire(HardwareInitSemaphoreHandle, osWaitForever);
  osSemaphoreRelease(HardwareInitSemaphoreHandle);
  osSemaphoreAcquire(UartInitSemaphoreHandle, 0);
  my_uart_init(&bluetooth);
  osSemaphoreRelease(UartInitSemaphoreHandle);
  for(;;)
  {
	if(my_uart_get_message(&bluetooth, received_message) == true){
	  parse_command(&command_context, received_message);
	}
    osDelay(50);
  }
  /* USER CODE END StartReceiveCmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */


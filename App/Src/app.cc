/**
 * @file     app.cc
 * @brief    Application code interface.
 * @author   Haoze Zhang
 * @version  20200211
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_nucleo_144.h"
#include "printf.h"
#include "stm32h7xx_hal.h"

#include "app.h"
#include "frequency_counter.h"

void _putchar(char c) {
	extern UART_HandleTypeDef huart3;
	HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(&c), 1, 10);
}

// Task: LEDToggle
static TaskHandle_t taskLEDToggleHandle = NULL;

static void taskLEDToggle(void* params) {
	for (;;) {
		TickType_t xLastWakeTime = xTaskGetTickCount();
	    BSP_LED_Toggle(LED_GREEN);
		vTaskDelayUntil(&xLastWakeTime, 50U);
	}
}

// Task: FreqCnt
static TaskHandle_t taskFreqCntHandle = NULL;
static const float tickFrequency = 100.0f*1.0e3f;
// Hamming window, n = 23, wc = 1, unit gain
static const float h[] = {
-0.0008080433472f, 4.342992899e-19f,  0.001889089821f, 0.006307432894f, 0.01449862681f,
 0.02704446949f,   0.04353854433f,    0.06249894574f,  0.08156600595f,  0.09795180708f,
 0.1090356931f,    0.1129548475f,     0.1090356931f,   0.09795180708f,  0.08156600595f,
 0.06249894574f,   0.04353854433f,    0.02704446949f,  0.01449862681f,  0.006307432894f,
 0.001889089821f,  4.342992899e-19f, -0.0008080433472f
};
static float x[sizeof(h)/sizeof(float)] = {0};
brown::FrequencyCounter<uint16_t, float> fc(
	sizeof(h)/sizeof(float), h, x, 100.0f*1.0e3f);

void taskFreqCntTimerISR(TIM_HandleTypeDef *htim) {
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
		fc.inputTick(htim->Instance->CCR1);
	} else if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
		fc.timeout();
	}
}

static void taskFreqCnt(void* params)
{
	for (;;) {
		TickType_t xLastWakeTime = xTaskGetTickCount();
		float f = fc.outputFrequency();
		printf_("Frequency = %5.1fHz\r\n",f);
		if (f < 0.0f) {
			BSP_LED_Toggle(LED_BLUE);
		}
		vTaskDelayUntil(&xLastWakeTime, 50U);
	}
}

BaseType_t appCreateTasks() {
	BaseType_t isSuccess = pdTRUE;

	/*
	 * Template for creating a task.
	 * xTaskCreate(
	 *     vTaskCode,        // Function that implements the task.
	 *     "NAME",           // Text name for the task.
	 *     100,              // Stack size in words, not bytes.
	 *     NULL,             // Parameter passed into the task.
	 *     tskIDLE_PRIORITY, // Priority at which the task is created.
   	 *     &xHandle);        // Used to pass out the created task's handle.
	 */

	isSuccess &= xTaskCreate(
		taskLEDToggle, "LEDToggle", 64, NULL,
		tskIDLE_PRIORITY+10, &taskLEDToggleHandle);

	isSuccess &= xTaskCreate(
		taskFreqCnt, "FreqCnt", 256, NULL,
		tskIDLE_PRIORITY+8, &taskFreqCntHandle);

	return isSuccess;
}

void appInit() {
	// Task: LEDToggle
	BSP_LED_Init(LED_GREEN);

	// Task: FreqCnt
	extern TIM_HandleTypeDef htim17;
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
}

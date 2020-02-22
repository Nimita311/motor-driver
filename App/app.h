/**
 * @file     app.h
 * @brief    Application code interface including initialization, task creation,
 *           and ISRs.
 * @author   Haoze Zhang
 * @version  20200211
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * Initialize the application.
  */
void appInit();

/**
  * Create all application level FreeRTOS tasks.
  * This function should be called in main before the kernel starts.
  * @return BaseType_t FreeRTOS success flag
  */
BaseType_t appCreateTasks();

void taskFreqCntTimerISR(TIM_HandleTypeDef *htim);
void _putchar(char c);
void _putblock(char* c, size_t size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* INC_APP_H_ */

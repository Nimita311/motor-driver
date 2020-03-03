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

#define APP_HARDFAULT_ISR                                           \
__asm volatile                                                      \
(                                                                   \
    " tst lr, #4                                                \n" \
    " ite eq                                                    \n" \
    " mrseq r0, msp                                             \n" \
    " mrsne r0, psp                                             \n" \
    " ldr r1, [r0, #24]                                         \n" \
    " ldr r2, handler2_address_const                            \n" \
    " bx r2                                                     \n" \
    " handler2_address_const: .word getRegistersHardFaultISR    \n" \
);

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
void taskTXUartISR();
void taskRXUartISR();
void getRegistersHardFaultISR(uint32_t* pStack);

void _putchar(char c);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* INC_APP_H_ */

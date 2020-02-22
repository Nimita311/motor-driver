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
#include "queue.h"
#include "stm32h7xx_nucleo_144.h"
#include "printf.h"
#include "stm32h7xx_hal.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "app.h"
#include "pid.hh"
#include "messenger.hh"
#include "freq_counter.hh"

#include "info.pb.h"
#include "cmd.pb.h"

/* Task: LEDToggle **********************************************************/
static TaskHandle_t taskLEDToggleHandle = NULL;
static TickType_t LEDOnDuration = pdMS_TO_TICKS(100);
static const TickType_t LEDTogglePeriod = pdMS_TO_TICKS(500);

static void taskLEDToggle(void* params) {
    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        BSP_LED_On(LED_GREEN);
        vTaskDelayUntil(&xLastWakeTime, LEDOnDuration);
        BSP_LED_Off(LED_GREEN);
        vTaskDelayUntil(&xLastWakeTime, LEDTogglePeriod);
    }
}

/* Task Msg *****************************************************************/
static TaskHandle_t taskMsgHandle = NULL;
static const UBaseType_t msgQueueSize = 10;
static QueueHandle_t msgQueue = xQueueCreate(
    msgQueueSize, sizeof(Info*));
static const size_t msgBufferSize = 1024;
ALIGN_32BYTES(static uint8_t msgBuffer[msgBufferSize]);
static brown::Messenger messenger(msgBuffer, msgBufferSize, _putblock);

static void taskMsg(void* params) {
    Info* pContainer;
    uint32_t id = 0;
    for (;;) {
        if (xQueueReceive(msgQueue, &pContainer, 10)) {
            pContainer->timestamp = xTaskGetTickCount();
            pContainer->id = id++;
            messenger.sendPerBlock(Info_fields, pContainer);
        }
    }
}

void taskMsgDMAISR() {
    messenger.sendCompleteCallback();
}

/* Task: FreqCnt ************************************************************/
static TaskHandle_t taskFreqCntHandle = NULL;
// 100kHz timer tick
static const float tickFrequency = 100.0f*1.0e3f;
// Hamming window, n = 23, wc = 1, unit gain
static const uint8_t filterOrder = 23;
static const float h[filterOrder] = {
    -0.0008080433472f, 4.342992899e-19f,  0.001889089821f, 0.006307432894f, 0.01449862681f,
    0.02704446949f,   0.04353854433f,    0.06249894574f,  0.08156600595f,  0.09795180708f,
    0.1090356931f,    0.1129548475f,     0.1090356931f,   0.09795180708f,  0.08156600595f,
    0.06249894574f,   0.04353854433f,    0.02704446949f,  0.01449862681f,  0.006307432894f,
    0.001889089821f,  4.342992899e-19f, -0.0008080433472f };
static float periods[filterOrder] = {0};
// 24 beats per revolution encoder
static const uint8_t encoderBPR = 24;
static uint16_t ticks[encoderBPR] = {0};

brown::FrequencyCounter<uint16_t, float, uint8_t> fc(
    h, periods, filterOrder, ticks, encoderBPR, tickFrequency);

void taskFreqCntTimerISR(TIM_HandleTypeDef *htim) {
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
        fc.input(htim->Instance->CCR1);
    } else if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
        fc.timeoutCallback();
    }
}

// static void taskFreqCnt(void* params){
//     for (;;) {
//         TickType_t xLastWakeTime = xTaskGetTickCount();
//         float f = fc.output();
//         printf_("Frequency = %6.3fHz\r\n",f);
//         vTaskDelayUntil(&xLastWakeTime, 50U);
//     }
// }

/* Task PID *****************************************************************/
static TaskHandle_t taskPIDHandle = NULL;
static const float kp = 1.0f;
static const float ki = 0.5f;
static const float kd = 0.0f;
static const float PIDPeriod = 0.05f;
static const float antiWindupMin = 0.2f;
static const float antiWindupMax = 1.0f;
static brown::PID<float> pid(kp, ki, kd, PIDPeriod);

static void taskPID(void* params) {
    static Info container = Info_init_zero;
    container.which_content = Info_pid_info_tag;
    static PIDInfo& msg = container.content.pid_info;
    static const TickType_t period = 5U;
    static const float w_real = 80.0f;
    pid.enableAntiWindup(antiWindupMin, antiWindupMax);
    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        msg.y_real = fc.output();
        msg.y = msg.y_real/40.0f;
        msg.e_real = w_real - msg.y_real;
        msg.e = w_real/40.0f - msg.y;
        msg.x = pid.output(msg.e);
        msg.anti_windup_active = pid.getIsAntiWindupActive();
        float duty = 1.0-duty;
        duty = duty > 1.0 ? 1.0 : duty;
        duty = duty < 0.0 ? 0.0 : duty;
        TIM2->CCR1 = static_cast<uint32_t>(TIM2->ARR*duty);

        xQueueSend(msgQueue, static_cast<void*>(&container), 0);
        vTaskDelayUntil(&xLastWakeTime, period);
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
        taskMsg, "Msg", 256, NULL,
        tskIDLE_PRIORITY+8, &taskMsgHandle);

    // isSuccess &= xTaskCreate(
    //     taskFreqCnt, "FreqCnt", 256, NULL,
    //     tskIDLE_PRIORITY+8, &taskFreqCntHandle);

    isSuccess &= xTaskCreate(
        taskPID, "PID", 128, NULL,
        tskIDLE_PRIORITY+12, &taskPIDHandle);

    return isSuccess;
}

void appInit() {
    // Task: LEDToggle
    BSP_LED_Init(LED_GREEN);

    // Task: FreqCnt
    // Frequency counting on PF7
    extern TIM_HandleTypeDef htim17;
    HAL_TIM_Base_Start_IT(&htim17);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

    // Task: PID
    // PWM output on PA0
    extern TIM_HandleTypeDef htim2;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    BSP_LED_Init(LED_BLUE);
}

void _putchar(char c) {
    extern UART_HandleTypeDef huart3;
    HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(&c), 1, 10);
}

void _putblock(char* c, size_t size) {
    for (size_t i = 0; i < size; i++) {
        _putchar(c[i]);
    }
    messenger.sendCompleteCallback();
}

void _putblockDMA(char* c, size_t size) {
    extern UART_HandleTypeDef huart3;
    HAL_UART_Transmit_DMA(&huart3, reinterpret_cast<uint8_t*>(c), size);
}

void getRegisters(uint32_t* pStack) {
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr;  /* Link register. */
	volatile uint32_t pc;  /* Program counter. */
	volatile uint32_t psr; /* Program status register. */

    r0 = pStack[ 0 ];
    r1 = pStack[ 1 ];
    r2 = pStack[ 2 ];
    r3 = pStack[ 3 ];

    r12 = pStack[ 4 ];
    lr = pStack[ 5 ];
    pc = pStack[ 6 ];
    psr = pStack[ 7 ];

	BSP_LED_Init(LED_RED);
	BSP_LED_On(LED_RED);

    for( ;; );
}

void configureTimerForRunTimeStats() {
    extern __IO uint32_t uwTick;
    uwTick = 0;
}

unsigned long getRunTimeCounterValue() {
    return HAL_GetTick();
}

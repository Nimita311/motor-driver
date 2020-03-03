/**
 * @file     app.cc
 * @brief    Application code interface.
 * @author   Haoze Zhang
 * @version  20200211
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */
#define STM32H743xx

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32h7xx_nucleo_144.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"

#include "printf.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "app.h"
#include "pid.hh"
#include "freq_counter.hh"
#include "messenger.hh"

#include "info.pb.h"
#include "cmd.pb.h"



/* Settings *****************************************************************/
#define TX_UART USART3
#define TX_DMA DMA1
#define TX_DMA_STREAM LL_DMA_STREAM_0
#define RX_UART USART3
#define RX_DMA DMA1
#define RX_DMA_STREAM LL_DMA_STREAM_1

/* Handles ******************************************************************/
static TaskHandle_t taskPIDHandle = NULL;

/* Static functions *********************************************************/
static void _putblock(char* c, size_t size);
static void _putblockDMA(char* c, size_t size);

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
        vTaskDelayUntil(&xLastWakeTime, LEDTogglePeriod-LEDOnDuration);
    }
}

/* Task TX ******************************************************************/
static TaskHandle_t taskTXHandle = NULL;
static const UBaseType_t msgQueueSize = 10;
static QueueHandle_t msgQueue = xQueueCreate(
    msgQueueSize, sizeof(Info*));
static const size_t msgBufferSize = 1024;
ALIGN_32BYTES(static uint8_t msgBuffer[msgBufferSize]);
static brown::Sender messenger(msgBuffer, msgBufferSize, _putblockDMA);

static void taskTX(void* params) {
    Info* pContainer;
    uint32_t id = 0;
    for (;;) {
        if (xQueueReceive(msgQueue, &pContainer, 10)) {
            pContainer->timestamp = xTaskGetTickCount();
            pContainer->id = id++;
            messenger.send(Info_fields, pContainer);
        }
    }
}

void taskTXUartISR() {
    // If UART transmission complete
    if (LL_USART_IsActiveFlag_TC(TX_UART)) {
        messenger.sendCompleteCallback();
        // Clear UART flag
        LL_USART_ClearFlag_TC(TX_UART);
        // Clear DMA IT flags, otherwise next transmission will not start.
        LL_DMA_ClearFlag_FE0(TX_DMA);
        LL_DMA_ClearFlag_HT0(TX_DMA);
        LL_DMA_ClearFlag_TC0(TX_DMA);
    }
}

/* Task: RX *****************************************************************/
static TaskHandle_t taskRXHandle = NULL;

static const size_t rxRawBufferSize = 1024;
static const size_t rxPktBufferSize = 512;
ALIGN_32BYTES(static uint8_t rxRawBuffer[rxRawBufferSize]);
ALIGN_32BYTES(static uint8_t rxPktBuffer[rxPktBufferSize]);

static brown::Receiver receiver(
    rxRawBuffer, rxRawBufferSize, rxPktBuffer, rxPktBufferSize);

static Cmd pidCmdContainer = Cmd_init_default;
static PIDCmd& pidcmd = pidCmdContainer.content.pid_cmd;

static void taskRX(void* params) {
    static Cmd cmdContainer = Cmd_init_default;

    for (;;) {
        // Use task notification as a binary semaphore. Block indefinitely.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Packet received
        if (!receiver.receive(Cmd_fields, &cmdContainer)) {
            // Ignore bad packet
            continue;
        }
        // Pass message to task
        if (cmdContainer.which_content == Cmd_pid_cmd_tag) {
        	memcpy(&pidCmdContainer, &cmdContainer, sizeof(cmdContainer));
            xTaskNotifyGive(taskPIDHandle);
        }
    }
}

void taskRXUartISR() {
    if (LL_USART_IsActiveFlag_RTO(RX_UART)) {
        receiver.receiveCompleteCallback(
            LL_DMA_GetDataLength(RX_DMA, RX_DMA_STREAM));
        LL_USART_ClearFlag_RTO(RX_UART);

        BaseType_t isTaskWoken = pdFALSE;
        // Notify RX transmission is complete
        vTaskNotifyGiveFromISR(taskRXHandle, &isTaskWoken);
        // Context switch to the woken task
        portYIELD_FROM_ISR(isTaskWoken);
    }
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

//     }
// }

/* Task PID *****************************************************************/
//static const float kp = 0.50f;
//static const float ki = 1.0f;
//static const float kd = 0.0f;

static void pidHandleCommand(brown::PID<float>& pid, PIDCmd& cmd) {
    if (!cmd.enable_pid) {return;}
    pid.setGains(cmd.kp, cmd.ki, cmd.kd, cmd.ts, cmd.ts);
    if (cmd.enable_anti_windup) {
        pid.enableAntiWindup(cmd.min, cmd.max);
    } else {
        pid.disableAntiWindup();
    }
}

static void taskPID(void* params) {
    static Info container = Info_init_zero;
    static Info* pContainer = &container;
    container.which_content = Info_pid_info_tag;
    static PIDInfo& msg = container.content.pid_info;
    static const TickType_t period = pdMS_TO_TICKS(50);

    pidcmd.kp = 3.0f;
    pidcmd.ki = 1.0f;
    pidcmd.kd = 0.0f;
    pidcmd.ts = 0.05f;
    pidcmd.min = 0.2f;
    pidcmd.max = 1.0f;
    pidcmd.enable_pid = false;
    pidcmd.enable_anti_windup = false;
    static brown::PID<float> pid;
    pidHandleCommand(pid, pidcmd);

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        float duty = 0.0f;
        msg.y_real = fc.output();
        if (pidcmd.enable_pid) {
            msg.y = msg.y_real/40.0f;
            msg.e_real = pidcmd.w - msg.y_real;
            msg.e = pidcmd.w/40.0f - msg.y;
            msg.x = pid.output(msg.e);
            msg.anti_windup_active = pid.getIsAntiWindupActive();
        } else {
            msg.x = pidcmd.u;
        }
        duty = msg.x;
        duty = duty > 1.0 ? 1.0 : duty;
        duty = duty < 0.0 ? 0.0 : duty;
        duty = duty*duty*-0.9920f + duty*1.8632 + 0.1155;
        msg.x_real = duty;
        TIM2->CCR1 = static_cast<uint32_t>(TIM2->ARR*duty);

        if (msg.anti_windup_active) {BSP_LED_On(LED_BLUE);}
        else {BSP_LED_Off(LED_BLUE);}

        // handle command
        if (ulTaskNotifyTake(pdTRUE, 0)) {
            pidHandleCommand(pid, pidcmd);
        }

        xQueueSend(msgQueue, static_cast<void*>(&pContainer), 0);
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
        taskLEDToggle, "LEDToggle", 128, NULL,
        tskIDLE_PRIORITY+10, &taskLEDToggleHandle);

    isSuccess &= xTaskCreate(
        taskTX, "TX", 512, NULL,
        tskIDLE_PRIORITY+8, &taskTXHandle);

    isSuccess &= xTaskCreate(
        taskRX, "RX", 512, NULL,
        tskIDLE_PRIORITY+10, &taskRXHandle);

    // isSuccess &= xTaskCreate(
    //     taskFreqCnt, "FreqCnt", 128, NULL,
    //     tskIDLE_PRIORITY+8, &taskFreqCntHandle);

    isSuccess &= xTaskCreate(
        taskPID, "PID", 512, NULL,
        tskIDLE_PRIORITY+12, &taskPIDHandle);


    return isSuccess;
}

// UART3 TX on DMA1 Stream0


void appInit() {
    // Task: LEDToggle
    BSP_LED_Init(LED_GREEN);

    // Task: Msg
    // Enable transmission complete IRQ
    LL_USART_ClearFlag_TC(TX_UART);
    LL_USART_EnableIT_TC(TX_UART);
    // Enable UART
    LL_USART_Enable(TX_UART);
    // Enable UART transmission
    LL_USART_EnableDirectionTx(TX_UART);
    // Enable UART DMA transmission request
    LL_USART_EnableDMAReq_TX(TX_UART);
    LL_DMA_SetPeriphAddress(TX_DMA, TX_DMA_STREAM,
            LL_USART_DMA_GetRegAddr(TX_UART, LL_USART_DMA_REG_DATA_TRANSMIT));

    // Enable receiver timeout IRQ
    LL_USART_ClearFlag_RTO(RX_UART);
    LL_USART_EnableIT_RTO(RX_UART);
    LL_USART_EnableRxTimeout(RX_UART);
    LL_USART_SetRxTimeout(RX_UART, 10000);
    // Enable UART receiving
    LL_USART_EnableDirectionRx(RX_UART);
    LL_USART_EnableDMAReq_RX(RX_UART);
    LL_DMA_SetPeriphAddress(RX_DMA, RX_DMA_STREAM,
    	LL_USART_DMA_GetRegAddr(RX_UART, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(RX_DMA, RX_DMA_STREAM,
    	reinterpret_cast<uint32_t>(rxRawBuffer));
    LL_DMA_SetDataLength(RX_DMA, RX_DMA_STREAM, rxRawBufferSize);
    LL_DMA_EnableStream(RX_DMA, RX_DMA_STREAM);

    // Task: FreqCnt
    // Frequency counting on TIM17 CH1: PF7
    extern TIM_HandleTypeDef htim17;
    HAL_TIM_Base_Start_IT(&htim17);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

    // Task: PID
    // PWM output on TIM2 CH1: PA0
    extern TIM_HandleTypeDef htim2;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    BSP_LED_Init(LED_BLUE);
}

void _putchar(char c) {
    LL_USART_TransmitData8(TX_UART, c);
}

// static void _putblock(char* c, size_t size) {
//     for (size_t i = 0; i < size; i++) {
//         _putchar(c[i]);
//     }
//     messenger.sendCompleteCallback();
// }

static void _putblockDMA(char* c, size_t size) {
    LL_DMA_SetDataLength(TX_DMA, TX_DMA_STREAM, size);
    LL_DMA_SetMemoryAddress(TX_DMA, TX_DMA_STREAM,
            reinterpret_cast<uint32_t>(c));
    // Enable DMA
    LL_DMA_EnableStream(TX_DMA, TX_DMA_STREAM);

}

void getRegistersHardFaultISR(uint32_t* pStack) {
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

//void configureTimerForRunTimeStats() {
//    extern __IO uint32_t uwTick;
//    uwTick = 0;
//}
//
//unsigned long getRunTimeCounterValue() {
//    return HAL_GetTick();
//}

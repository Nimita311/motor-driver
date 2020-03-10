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

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32h7xx_nucleo_144.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"

#include "app_serv/led_service.hh"
#include "app_serv/rx_service.hh"
#include "app_serv/tx_service.hh"
#include "app_serv/bldc_service.hh"

#include "app_msg/info.pb.h"
#include "app_msg/cmd.pb.h"

/* Configurations ***********************************************************/
static USART_TypeDef* const rxUart = USART3;
static DMA_TypeDef* const rxDma = DMA1;
static const uint32_t rxStream = LL_DMA_STREAM_1;
constexpr static const size_t rxRawBufferSize = 1024;
constexpr static const size_t rxPktBufferSize = 512;

static USART_TypeDef* const txUart = USART3;
static DMA_TypeDef* const txDma = DMA1;
static const uint32_t txStream = LL_DMA_STREAM_0;
constexpr static const size_t txQueueSize = 10;
constexpr static const size_t txBufferSize = 1024;

// 100kHz
constexpr static const float encoderTimerTickFrequency = 100.0f*1.0e3f;
// Hamming window, n = 23, wc = 1, unit gain
constexpr static const uint8_t encoderFilterOrder = 23;
// 24 beats per revolution encoder
constexpr static const uint8_t encoderBPR = 24;

/* Services *****************************************************************/
static brown::LEDService led(LED_GREEN, LED_BLUE);

ALIGN_32BYTES(static uint8_t rxRawBuffer[rxRawBufferSize]);
ALIGN_32BYTES(static uint8_t rxPktBuffer[rxPktBufferSize]);
static brown::RXService rx(
    rxRawBuffer, rxRawBufferSize, rxPktBuffer, rxPktBufferSize,
    rxUart, rxDma, rxStream);

ALIGN_32BYTES(static uint8_t txBuffer[txBufferSize]);
static void _putblockDMA(char* c, size_t size) {
    LL_DMA_SetDataLength(txDma, txStream, size);
    LL_DMA_SetMemoryAddress(txDma, txStream, reinterpret_cast<uint32_t>(c));
    // Enable DMA
    LL_DMA_EnableStream(txDma, txStream);
}
void _putchar(char c) {
    LL_USART_TransmitData8(txUart, c);
}
static brown::TXService tx(
    txBuffer, txBufferSize, _putblockDMA, txQueueSize,
    txUart, txDma, txStream);

static const float h[encoderFilterOrder] = {
   -0.0008080433472f, 4.342992899e-19f,  0.001889089821f, 0.006307432894f, 0.01449862681f,
    0.02704446949f,   0.04353854433f,    0.06249894574f,  0.08156600595f,  0.09795180708f,
    0.1090356931f,    0.1129548475f,     0.1090356931f,   0.09795180708f,  0.08156600595f,
    0.06249894574f,   0.04353854433f,    0.02704446949f,  0.01449862681f,  0.006307432894f,
    0.001889089821f,  4.342992899e-19f, -0.0008080433472f };
static float periods[encoderFilterOrder] = {0};
static uint16_t ticks[encoderBPR] = {0};
brown::BLDCService<uint16_t, float, uint8_t> bldc(
    h, periods, encoderFilterOrder, ticks,
    encoderBPR, encoderTimerTickFrequency, tx);

/* C Interfaces *************************************************************/
bool appStartServices() {
    return led.start("led", 128, 1)
        && tx.start("tx", 512, 2)
        && rx.start("rx", 512, 3)
        && bldc.start("bldc", 512, 4);
}

bool appInit() {
    extern TIM_HandleTypeDef htim17;
    extern TIM_HandleTypeDef htim2;
    bldc.setHardware(&htim17, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_1);
    return brown::Service::initAll();
}

void txUartISR() {
    tx.uartISR();
}

void rxUartISR() {
    rx.uartISR();
}

void bldcISR() {
    bldc.encoderTimerISR();
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

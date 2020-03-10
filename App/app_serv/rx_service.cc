#include "rx_service.hh"

namespace brown {

static const uint32_t rxTimeout = 8;

bool RXService::init() {
    /* 1. Configure UART receiver timeout */
    // Clear flag
    LL_USART_ClearFlag_RTO(uart);
    // Enable receiver timeout IRQ
    LL_USART_EnableIT_RTO(uart);
    LL_USART_EnableRxTimeout(uart);
    // Set timeout value to minimum
    LL_USART_SetRxTimeout(uart, rxTimeout);

    /* 2. Configure UART DMA receiving */
    LL_USART_EnableDirectionRx(uart);
    LL_USART_EnableDMAReq_RX(uart);
    LL_DMA_SetPeriphAddress(dma, stream,
        LL_USART_DMA_GetRegAddr(uart, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(dma, stream,
        reinterpret_cast<uint32_t>(pRawBuffer));
    LL_DMA_SetDataLength(dma, stream, rawBufferSize);
    LL_DMA_EnableStream(dma, stream);
}

void RXService::run() {
static Cmd cmdContainer = Cmd_init_default;
for (;;) {
    // Use task notification as a binary semaphore. Block indefinitely.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Packet received
    if (!receive(Cmd_fields, &cmdContainer)) {
        // Ignore bad packet
        continue;
    }
    // Pass command to all services
    for (Service* s = Service::getHead(); s != nullptr; s=s->getNext()) {
        s->command(cmdContainer);
    }
}
}

void RXService::uartISR() {
    if (LL_USART_IsActiveFlag_RTO(uart)) {
        receiveCompleteCallback(LL_DMA_GetDataLength(dma, stream));
        LL_USART_ClearFlag_RTO(uart);

        BaseType_t isTaskWoken = pdFALSE;
        // Notify RX transmission is complete
        vTaskNotifyGiveFromISR(taskHandle, &isTaskWoken);
        // Context switch to the woken task
        portYIELD_FROM_ISR(isTaskWoken);
    }
}

} // namespace brown

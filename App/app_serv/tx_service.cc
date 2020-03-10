#include "app_serv/tx_service.hh"
#include "app_msg/info.pb.h"

namespace brown {

bool TXService::init() {
    // Enable transmission complete IRQ
    LL_USART_ClearFlag_TC(uart);
    LL_USART_EnableIT_TC(uart);
    // Enable UART
    LL_USART_Enable(uart);
    // Enable UART transmission
    LL_USART_EnableDirectionTx(uart);
    // Enable UART DMA transmission request
    LL_USART_EnableDMAReq_TX(uart);
    LL_DMA_SetPeriphAddress(dma, stream,
        LL_USART_DMA_GetRegAddr(uart, LL_USART_DMA_REG_DATA_TRANSMIT));
}

void TXService::run() {
Info* pInfoContainer;
uint32_t id = 0;
for (;;) {
    if (xQueueReceive(txQueue, &pInfoContainer, portMAX_DELAY)) {
        pInfoContainer->timestamp = xTaskGetTickCount();
        pInfoContainer->id = id++;
        Sender::send(Info_fields, pInfoContainer);
    }
}
}

void TXService::send(Info* pInfoContainer) {
    xQueueSend(txQueue, static_cast<void*>(&pInfoContainer), 0);
}

void TXService::uartISR() {
    // If UART transmission complete
    if (LL_USART_IsActiveFlag_TC(uart)) {
        sendCompleteCallback();
        // Clear UART flag
        LL_USART_ClearFlag_TC(uart);
        // Clear DMA IT flags, otherwise next transmission will not start.
        LL_DMA_ClearFlag_FE0(dma);
        LL_DMA_ClearFlag_HT0(dma);
        LL_DMA_ClearFlag_TC0(dma);
    }
}

} // namespace brown

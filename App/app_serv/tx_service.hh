#ifndef _INC_TX_SERVICE_HH
#define _INC_TX_SERVICE_HH

#include "app_serv/service.hh"
#include "app_lib/messenger.hh"
#include "app_msg/info.pb.h"

#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"

#include "queue.h"

namespace brown {

class TXService: public Service, public Sender {
private:
    const UBaseType_t txQueueSize;
    QueueHandle_t txQueue;

hardware:
    USART_TypeDef* const uart;
    DMA_TypeDef* const dma;
    const uint32_t stream;

public:
    TXService(uint8_t* txBuffer, size_t txBufferSize,
              void (*putblock) (char*, size_t),
              UBaseType_t txQueueSize,
              USART_TypeDef* uart, DMA_TypeDef* dma, uint32_t stream):
        Sender(txBuffer, txBufferSize, putblock),
        txQueueSize(txQueueSize),
        uart(uart), dma(dma), stream(stream) {
        txQueue = xQueueCreate(txQueueSize, sizeof(Info*));
    }
    bool init();
    void run();

async:
    void send(Info* pInfoContainer);

isr:
    void uartISR();
};

} // namespace brown

#endif // _INC_TX_SERVICE_HH

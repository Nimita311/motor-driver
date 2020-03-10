#ifndef _INC_RX_SERVICE_HH
#define _INC_RX_SERVICE_HH

#include "app_serv/service.hh"
#include "app_lib/messenger.hh"

#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"


namespace brown {

class RXService: public Service, public Receiver {
hardware:
    USART_TypeDef* const uart;
    DMA_TypeDef* const dma;
    const uint32_t stream;

public:
    RXService(uint8_t* pRawBuffer, size_t rawBufferSize,
              uint8_t* pPktBuffer, size_t pktBufferSize,
              USART_TypeDef* uart, DMA_TypeDef* dma, uint32_t stream):
        Receiver(pRawBuffer, rawBufferSize, pPktBuffer, pktBufferSize),
        uart(uart), dma(dma), stream(stream) {}
    bool init();
    void run();

isr:
    void uartISR();
};

} // namespace brown

#endif // _INC_RX_SERVICE_HH

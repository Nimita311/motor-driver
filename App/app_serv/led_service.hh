#ifndef _INC_LED_SERVICE_HH
#define _INC_LED_SERVICE_HH

#include "service.hh"
#include "stm32h7xx_nucleo_144.h"

namespace brown {

class LEDService: public Service {
private:
    TickType_t period = 100;
    TickType_t onTime = 50;

hardware:
    Led_TypeDef blinker;
    Led_TypeDef indicator;

public:
    LEDService(Led_TypeDef blinker, Led_TypeDef indicator):
        blinker(blinker), indicator(indicator) {}
    bool init();
    void run();

async:
    inline void setBlinkerPeriod(uint32_t ms) {period = pdMS_TO_TICKS(ms);}
    inline void setBlinkerOnTime(uint32_t ms) {onTime = pdMS_TO_TICKS(ms);}
    inline void setBlinkerDuty(uint8_t duty) {onTime = period*duty/100U;}

sync:
    inline void setIndicatorOn() {BSP_LED_On(indicator);}
    inline void setIndicatorOff() {BSP_LED_Off(indicator);}
};

} // namespace brown

#endif // _INC_LED_SERVICE_HH

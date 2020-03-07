#include "led_service.hh"

namespace brown {

bool LEDService::init() {
    BSP_LED_Init(blinker);
    BSP_LED_Init(indicator);
}

void LEDService::run() {
for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BSP_LED_On(blinker);
    vTaskDelayUntil(&xLastWakeTime, onTime);
    BSP_LED_Off(blinker);
    vTaskDelayUntil(&xLastWakeTime, period-onTime);
}
}

} // namespace brown

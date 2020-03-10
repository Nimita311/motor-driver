#ifndef _INC_BLDC_SERVICE_HH
#define _INC_BLDC_SERVICE_HH

#include "app_serv/service.hh"
#include "app_serv/tx_service.hh"
#include "app_lib/pid.hh"
#include "app_lib/freq_counter.hh"
#include "app_msg/cmd.pb.h"

#include "stm32h7xx_hal.h"

namespace brown {

template <class TT, class TF, class SIZE>
class BLDCService: public Service {
private:
    FrequencyCounter<TT, TF, SIZE> fc;
    PID<TF> pid;
    TXService& tx;
    TickType_t pidPeroidTick;
    Cmd cmdContainer;
    Info infoContainer;
    void _pidApplyCommand();

hardware:
    TIM_HandleTypeDef* encoderTimer;
    uint32_t encoderChannel;
    TIM_HandleTypeDef* pwmTimer;
    uint32_t pwmChannel;

public:
    BLDCService(
        const TF h[], TF periods[], SIZE nP,
        TT ticks[], SIZE nT, TF tickFrequency, TXService& tx):
        fc(h, periods, nP, ticks, nT, tickFrequency), tx(tx) {
        cmdContainer = Cmd_init_default;
        infoContainer = Info_init_default;
        infoContainer.which_content = Info_pid_info_tag;
    }
    void setHardware(
        TIM_HandleTypeDef* encoderTimer, uint32_t encoderChannel,
        TIM_HandleTypeDef* pwmTimer, uint32_t pwmChannel);
    bool init();
    void run();
    void command(Cmd& cmdContainer);

isr:
    void encoderTimerISR();
};

} // namespace brown

#endif // _INC_TX_SERVICE_HH

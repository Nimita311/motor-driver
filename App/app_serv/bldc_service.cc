#include "app_serv/bldc_service.hh"

namespace brown {

static const uint32_t defaultPidPeriodMs = 50;

template <class TT, class TF, class SIZE>
void BLDCService<TT, TF, SIZE>::setHardware(
    TIM_HandleTypeDef* encoderTimer, uint32_t encoderChannel,
    TIM_HandleTypeDef* pwmTimer, uint32_t pwmChannel) {
    this->encoderTimer = encoderTimer;
    this->encoderChannel = encoderChannel;
    this->pwmTimer = pwmTimer;
    this->pwmChannel = pwmChannel;
}

template <class TT, class TF, class SIZE>
void BLDCService<TT, TF, SIZE>::_pidApplyCommand() {
    PIDCmd& cmd = cmdContainer.content.pid_cmd;
    if (!cmd.enable_pid) {
        pidPeroidTick = pdMS_TO_TICKS(defaultPidPeriodMs);
        return;
    } else {
        pid.setGains(cmd.kp, cmd.ki, cmd.kd, cmd.ts, cmd.ts);
        pidPeroidTick = pdMS_TO_TICKS(cmd.ts*1000);
    }
    if (cmd.enable_anti_windup) {
        pid.enableAntiWindup(cmd.min, cmd.max);
    } else {
        pid.disableAntiWindup();
    }
}

template <class TT, class TF, class SIZE>
bool BLDCService<TT, TF, SIZE>::init() {
    // Frequency counting on TIM17 CH1: PF7
    HAL_TIM_Base_Start_IT(encoderTimer);
    HAL_TIM_IC_Start_IT(encoderTimer, encoderChannel);
    // PWM output on TIM2 CH1: PA0
    HAL_TIM_PWM_Start(pwmTimer, pwmChannel);
}

template <class TT, class TF, class SIZE>
void BLDCService<TT, TF, SIZE>::run() {
static Info* pInfoContainer = &infoContainer;
static PIDInfo& pidInfo = infoContainer.content.pid_info;
static PIDCmd& pidCmd = cmdContainer.content.pid_cmd;
pidCmd.enable_pid = false;
pidCmd.enable_anti_windup = false;
_pidApplyCommand();
for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float duty = 0.0f;
    pidInfo.y_real = fc.output();
    if (pidCmd.enable_pid) {
        pidInfo.y = pidInfo.y_real/40.0f;
        pidInfo.e_real = pidCmd.w - pidInfo.y_real;
        pidInfo.e = pidCmd.w/40.0f - pidInfo.y;
        pidInfo.x = pid.output(pidInfo.e);
        pidInfo.anti_windup_active = pid.getIsAntiWindupActive();
    } else {
        pidInfo.x = pidCmd.u;
    }
    duty = pidInfo.x;
    duty = duty > 1.0 ? 1.0 : duty;
    duty = duty < 0.0 ? 0.0 : duty;
    duty = duty*duty*-0.9920f + duty*1.8632 + 0.1155;
    pidInfo.x_real = duty;
    pwmTimer->Instance->CCR1 =
        static_cast<uint32_t>(pwmTimer->Instance->ARR*duty);

    // Received any pending command
    if (ulTaskNotifyTake(pdTRUE, 0)) {
        _pidApplyCommand();
    }

    // Send PID Info
    tx.send(pInfoContainer);

    // Block until next period
    vTaskDelayUntil(&xLastWakeTime, pidPeroidTick);
}
}

template <class TT, class TF, class SIZE>
void BLDCService<TT, TF, SIZE>::command(Cmd& cmdContainer) {
    if (cmdContainer.which_content == Cmd_pid_cmd_tag) {
        memcpy(&this->cmdContainer, &cmdContainer, sizeof(cmdContainer));
        xTaskNotifyGive(taskHandle);
    }
}

template <class TT, class TF, class SIZE>
void BLDCService<TT, TF, SIZE>::encoderTimerISR() {
    if (__HAL_TIM_GET_FLAG(encoderTimer, TIM_FLAG_CC1) != RESET) {
        fc.input(encoderTimer->Instance->CCR1);
    } else if (__HAL_TIM_GET_FLAG(encoderTimer, TIM_FLAG_UPDATE) != RESET) {
        fc.timeoutCallback();
    }
}

} // namespace brown

// Explicitly instantiate `BLDCService`.
template class brown::BLDCService<uint16_t, float, uint8_t>;

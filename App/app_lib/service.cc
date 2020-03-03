#include "service.hh"

namespace brown {

void Service::_runService(void* service) {
    static_cast<Service*>(service)->run();
}

bool Service::start(const char* name,
                    const configSTACK_DEPTH_TYPE stackSize,
                    const UBaseType_t priority) {
    this->id = Service::serviceCount++;
    return xTaskCreate(
        _runService, name, stackSize, this,
        tskIDLE_PRIORITY+priority, &(this->taskHandle));
}

} // namespace brown

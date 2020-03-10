#include "service.hh"

namespace brown {

void Service::_runService(void* service) {
    static_cast<Service*>(service)->run();
}

bool Service::initAll() {
    bool isSuccess = true;
    for (Service* s = Service::head; s != nullptr; s=s->next) {
        isSuccess &= s->init();
    }
}

Service::Service() {
    // Add `this` to the back of the linked list
    if (Service::serviceCount == 0) {
        Service::head = this;
    } else {
        Service* s = Service::head;
        for (uint32_t i=0; i<serviceCount-1; i++) {
            s = s->next;
        }
        s->next = this;
    }
    this->id = Service::serviceCount++;
}

bool Service::start(const char* name,
                    const configSTACK_DEPTH_TYPE stackSize,
                    const UBaseType_t priority) {
    /*
     * Template for creating a task.
     * xTaskCreate(
     *     vTaskCode,        // Function that implements the task.
     *     "NAME",           // Text name for the task.
     *     100,              // Stack size in words, not bytes.
     *     NULL,             // Parameter passed into the task.
     *     tskIDLE_PRIORITY, // Priority at which the task is created.
     *     &xHandle);        // Used to pass out the created task's handle.
     */
    return pdPASS == xTaskCreate(
        _runService, name, stackSize, this,
        tskIDLE_PRIORITY+priority, &(this->taskHandle));
}

} // namespace brown

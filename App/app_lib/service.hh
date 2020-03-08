#ifndef INC_SERVICE_HH_
#define INC_SERVICE_HH_

#include "FreeRTOS.h"
#include "task.h"

#include "app_msg/cmd.pb.h"

#define hardware private
#define async public
#define sync public

namespace brown {

class Service {
private:
    static uint32_t serviceCount;
    static void _runService(void* service);

protected:
    uint32_t id;
    TaskHandle_t taskHandle = nullptr;

public:
    static void resetCounter() {serviceCount=0;}
    inline uint32_t getID() {return id;}
    inline TaskHandle_t getHandle() {return taskHandle;}

    /**
     * @brief Create the RTOS task for this service.
     * @param name Task name.
     * @param stackSize Size of stack in words.
     * @param priority Task priority.
     *
     * The target static function is `_runService` which calls `this->run`.
     */
    bool start(const char* name,
               const configSTACK_DEPTH_TYPE stackSize,
               const UBaseType_t priority);

    /**
     * @brief Service initialization.
     *
     * Pure virtual. Must override.
     */
    virtual bool init() = 0;

    /**
     * @brief Service infinite loop.
     *
     * Pure virtual. Must override. Must be an infinite loop and should not
     * return. This method should not be invoked directly.
     */
    virtual void run() = 0;

    /**
     * @brief Receive command.
     */
    virtual void receive(Cmd* pCmdContainer) {};
}; // class Service

} // namespace brown

#endif // INC_SERVICE_HH_

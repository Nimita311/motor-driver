#ifndef INC_SERVICE_HH_
#define INC_SERVICE_HH_

#include "FreeRTOS.h"
#include "task.h"

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

    bool start(const char* name,
               const configSTACK_DEPTH_TYPE stackSize,
               const UBaseType_t priority);
    inline uint32_t getID() {return id;}
    inline TaskHandle_t getHandle() {return taskHandle;}


    virtual bool init() = 0;
    virtual void run() = 0;
}; // class Service

} // namespace brown

#endif // INC_SERVICE_HH_

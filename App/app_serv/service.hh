#ifndef INC_SERVICE_HH_
#define INC_SERVICE_HH_

#include "FreeRTOS.h"
#include "task.h"

#include "app_msg/cmd.pb.h"

#define hardware private
#define async public
#define sync public
#define isr public

namespace brown {

class Service {
private:
    inline static Service* head = nullptr;
    inline static uint32_t serviceCount = 0;
    static void _runService(void* service);
    Service* next = nullptr;

protected:
    uint32_t id;
    TaskHandle_t taskHandle = nullptr;

public:
    static void reset() {head=nullptr; serviceCount=0;}
    static Service* getHead() {return head;}
    static bool initAll();

    inline Service* getNext() const {return next;}
    inline uint32_t getID() const {return id;}
    inline TaskHandle_t getHandle() const {return taskHandle;}

    Service();

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
     *
     * Not all services need to receive commands thus not pure virtual.
     */
    virtual void command(Cmd& cmdContainer) {};
}; // class Service

} // namespace brown

#endif // INC_SERVICE_HH_

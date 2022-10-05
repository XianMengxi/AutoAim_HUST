#ifndef HANDLER_H
#define HANDLER_H
#include"dynamicdraw.h"
class Handler
{
    DynamicDraw * handle_drawer;
        public:
    Handler(DynamicDraw * drawer)
    {
        handle_drawer = drawer;
    }
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    const std::string& chan,
                    const exlcm::example_t* msg)
    {
        handle_drawer->addData(msg->pose[0]);
    }
};

#endif // HANDLER_H

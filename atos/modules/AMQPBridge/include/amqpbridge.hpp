#pragma once

#include "module.hpp"
#include "amqpclient.hpp"

class AmqpBridge : public Module
{
private:
    static inline std::string const moduleName = "amqp_bridge";
    AmqpClientTmp amqpClient;
    /* data */
public:
    AmqpBridge(/* args */);
    ~AmqpBridge();
};

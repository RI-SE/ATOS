#include <SimpleAmqpClient/SimpleAmqpClient.h>

class AmqpClientTmp {
private:
    AmqpClient::Channel::ptr_t connection;
    /* data */
    AmqpClient::Envelope::ptr_t envelope;
    std::string consumerTag;

public:
    AmqpClientTmp();
};
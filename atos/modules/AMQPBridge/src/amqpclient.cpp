#include "amqpclient.hpp"
#include <SimpleAmqpClient/SimpleAmqpClient.h>

using namespace std;

AmqpClientTmp::AmqpClientTmp() {
    const std::string& uriString = "amqp://guest:guest@localhost:5672/";
    AmqpClient::Channel::OpenOpts options = AmqpClient::Channel::OpenOpts::FromUri(uriString);
    connection = AmqpClient::Channel::Open(options);
    // connection->DeclareQueue("my_queue");


    // connection->BasicPublish("", "my_queue", AmqpClient::BasicMessage::Create("Hello, world!"));
    // AmqpClient::BasicMessage::ptr_t msg = connection->BasicConsumeMessage("my_queue");


    // consumerTag = "my_queue";
    // std::string consumerId = channel->BasicConsume("my_queue", "");
    // Envelope::ptr_t envelope = channel->BasicConsumeMessage(consumerId);
    
    // // To ack:
    // channel->BasicAck(envelope);
    // // To cancel:
    // channel->BasicCancel(consumer_tag);
}
#include "amqpbridge.hpp"


AmqpBridge::AmqpBridge(/* args */) : Module(AmqpBridge::moduleName){
    amqpClient = AmqpClientTmp();

}

AmqpBridge::~AmqpBridge() {

}
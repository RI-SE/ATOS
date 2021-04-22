#include "objectlistener.hpp"

ObjectListener::ObjectListener(
		ScenarioHandler* sh,
		TestObject* ob)
	:  obj(ob), handler(sh)
{
	if (!obj->isConnected()) {
		throw std::invalid_argument("Attempted to start listener for disconnected object");
	}
	LogMessage(LOG_LEVEL_DEBUG, "Starting listener thread for object %u", ob->getTransmitterID());
	listener = std::thread(&ObjectListener::listen, this);
}

ObjectListener::~ObjectListener() {
	this->quit = true;
	LogMessage(LOG_LEVEL_DEBUG, "Awaiting thread exit");
	listener.join(); // TODO this blocks if MONR timeout
}

void ObjectListener::listen() {
	try {
		while (!this->quit) {
			switch (obj->pendingMessageType(true)) {
			case MESSAGE_ID_MONR: {
				auto monr = obj->readMonitorMessage();
				//
				LogPrint("Received MONR!");
				break;
			}
			case MESSAGE_ID_TREO:
				// TODO
				break;
			case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO:
				obj->parseObjectPropertyMessage();
				LogPrint("Received OPRO!");
				// TODO
				break;
			default:
				LogPrint("Received something!");
				// TODO
				break;
			}
		}
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, e.what());
	} catch (std::ios_base::failure& e) {
		LogMessage(LOG_LEVEL_ERROR, e.what());
		obj->disconnect();
		handler->state->disconnectedFromObject(*handler, obj->getTransmitterID());
	}
}

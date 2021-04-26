#include "objectlistener.hpp"
#include "datadictionary.h"
#include "maestroTime.h"

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
				struct timeval currentTime;
				auto monr = obj->readMonitorMessage();
				TimeSetToCurrentSystemTime(&currentTime);
				if (handler->controlMode == ScenarioHandler::RELATIVE_KINEMATICS && obj->isAnchor()) {
					// TODO transform
				}
				else {
					// TODO
				}
				DataDictionarySetMonitorData(monr.first, &monr.second, &currentTime); // TODO move to abs_kin
				break;
			}
			case MESSAGE_ID_TREO:
				LogMessage(LOG_LEVEL_WARNING, "Unhandled TREO message");
				break;
			case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO:
				obj->parseObjectPropertyMessage();
				break;
			default:
				LogMessage(LOG_LEVEL_WARNING, "Received unknown message type");
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

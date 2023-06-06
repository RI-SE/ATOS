/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectlistener.hpp"
#include "objectcontrol.hpp"
#include "state.hpp"

ObjectListener::ObjectListener(
		ObjectControl* sh,
		std::shared_ptr<TestObject> ob,
		rclcpp::Logger log)
	:  obj(ob), handler(sh),
	 Loggable(log)
{
	if (!obj->isConnected()) {
		throw std::invalid_argument("Attempted to start listener for disconnected object");
	}
	RCLCPP_DEBUG(get_logger(), "Starting listener thread for object %u", ob->getTransmitterID());
	listener = std::thread(&ObjectListener::listen, this);
}

ObjectListener::~ObjectListener() {
	this->quit = true;
	RCLCPP_DEBUG(get_logger(), "Awaiting thread exit");

	// Interrupt socket to unblock readMonitorMessage, and allow thread to exit gracefully
	obj->interruptSocket();
	listener.join();
	RCLCPP_DEBUG(get_logger(), "Thread exited");
}

void ObjectListener::listen() {
	try {
		while (!this->quit) {
			//handle incoming iso22133 messages 
			obj->handleISOMessage(true);
		}
	} catch (std::invalid_argument& e) {
		RCLCPP_ERROR(get_logger(), e.what()); // TODO: add comment explaining this case..
	} catch (std::range_error& e){
		RCLCPP_DEBUG(get_logger(), e.what()); // Socket was interrupted intentionally, exit gracefully
	} catch (std::runtime_error& e) {
		RCLCPP_ERROR(get_logger(), e.what());
		obj->disconnect();
		handler->state->disconnectedFromObject(*handler, obj->getTransmitterID());
	}
	RCLCPP_INFO(get_logger(), "Listener thread for object %u exiting", obj->getTransmitterID());
}
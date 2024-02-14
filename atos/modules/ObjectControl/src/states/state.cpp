/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"
#include "type.h"
#include <exception>

void ObjectControlState::setState(
		ObjectControl& handler,
		ObjectControlState *st) {
	// Lock stateMutex to prevent data races when ObjectListener threads execute handler methods
	std::lock_guard<std::mutex> lock(handler.stateMutex);

	// Before replacing state, execute any exit behaviour
	handler.state->onExit(handler);
	RCLCPP_INFO(handler.get_logger(), "Transitioning to state %s", type(*st).c_str());
	// Store state in a temporary variable to avoid handler.state being null at any point
	ObjectControlState* temp = handler.state;
	handler.state = st;
	delete temp;
	// After replacing state, execute any enter behaviour
	handler.state->onEnter(handler);
}



// Function bodies for pure virtual functions
void ObjectControlState::initializeRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disconnectRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::armRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disarmRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::startRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::stopRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::abortRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allClearRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::enableRemoteControlRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disableRemoteControlRequest(ObjectControl&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}

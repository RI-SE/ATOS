#include "state.hpp"
#include "logging.h"
#include "type.h"
#include "datadictionary.h"
#include <exception>

void ObjectControlState::setState(
		ScenarioHandler& handler,
		ObjectControlState *st) {
	// TODO mutex on state modification
	// Before replacing state, execute any exit behaviour
	handler.state->onExit(handler);
	LogMessage(LOG_LEVEL_INFO, "Transitioning to state %s", type(*st).c_str());
	// Store state in a temporary variable to avoid handler.state being null at any point
	ObjectControlState* temp = handler.state;
	handler.state = st;
	delete temp;
	DataDictionarySetOBCState(handler.state->asNumber());
	// After replacing state, execute any enter behaviour
	handler.state->onEnter(handler);
}



// Function bodies for pure virtual functions
void ObjectControlState::initializeRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disconnectRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::armRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disarmRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::startRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::stopRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::abortRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allClearRequest(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}


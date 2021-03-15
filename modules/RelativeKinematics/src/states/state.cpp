#include "state.hpp"
#include "logging.h"
#include <typeinfo>
#include <exception>

void ObjectControlState::setState(
		ScenarioHandler& handler,
		ObjectControlState *st) {
	LogMessage(LOG_LEVEL_INFO, "Transitioning to state %s", typeid (*st).name());
	ObjectControlState* temp = handler.state;
	handler.state = st;
	delete temp;
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
void ObjectControlState::connectedToObject(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disconnectedFromObject(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectedToLiveObject(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectedToArmedObject(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allObjectsDisarmed(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allObjectsConnected(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::testCompleted(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::postProcessingCompleted(ScenarioHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}

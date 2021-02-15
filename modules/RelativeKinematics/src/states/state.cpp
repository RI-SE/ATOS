#include "state.hpp"
#include "logging.h"
#include <typeinfo>
#include <exception>

void ObjectControlState::setState(
		ObjectHandler& handler,
		ObjectControlState *st) {
	LogMessage(LOG_LEVEL_INFO, "Transitioning to state %s", typeid (*st).name());
	ObjectControlState* temp = handler.state;
	handler.state = st;
	delete temp;
}



// Function bodies for pure virtual functions
// TODO: any subclass independent implementations may be placed in these and
// called from respective subclass
void ObjectControlState::initializeRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disconnectRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::armRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disarmRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::startRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::stopRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::abortRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allClearRequest(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectedToObject(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::disconnectedFromObject(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectedToLiveObject(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::connectedToArmedObject(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allObjectsDisarmed(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::allObjectsConnected(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::testCompleted(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}
void ObjectControlState::postProcessingCompleted(ObjectHandler&) {
	throw std::logic_error("Call to pure virtual function " + std::string(__FUNCTION__));
}

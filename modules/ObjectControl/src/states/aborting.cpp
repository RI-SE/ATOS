#include "state.hpp"

AbstractKinematics::Aborting::Aborting() {

}

void AbstractKinematics::Aborting::onExit(
		ObjectControl&) {
}

void AbstractKinematics::Aborting::allClearRequest(
		ObjectControl& handler) {
		handler.allClearObjects();
}

void AbstractKinematics::Aborting::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::connectedToLiveObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::objectAborting(
		ObjectControl& ,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::objectAbortDisarmed(
	ObjectControl& ,
	uint32_t) {
	// TODO
}

void AbstractKinematics::Aborting::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	LogMessage(LOG_LEVEL_WARNING, "Object disarmed while expecting abort");
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}

void AbstractKinematics::Aborting::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	LogMessage(LOG_LEVEL_WARNING, "Object armed while expecting abort");
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}


void AbstractKinematics::Aborting::allObjectsAbortDisarmed(
		ObjectControl& handler) {
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Aborting::allClearRequest(
		ObjectControl& handler) {
	AbstractKinematics::Aborting::allClearRequest(handler);
	setState(handler, new RelativeKinematics::Ready);
}

void RelativeKinematics::Aborting::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::connectedToLiveObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Aborting::objectAborting(handler, id);
	// TODO
}

void RelativeKinematics::Aborting::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Aborting::objectAbortDisarmed(handler,id);
}

void RelativeKinematics::Aborting::allObjectsAbortDisarmed(
		ObjectControl& handler) {
	AbstractKinematics::Aborting::allObjectsAbortDisarmed(handler);
	setState(handler, new RelativeKinematics::Ready);			
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Aborting::allClearRequest(
		ObjectControl& handler) {
	AbstractKinematics::Aborting::allClearRequest(handler);
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::Aborting::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::connectedToLiveObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Aborting::objectAborting(handler, id);
	// TODO
}

void AbsoluteKinematics::Aborting::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Aborting::objectAbortDisarmed(handler,id);
}

void AbsoluteKinematics::Aborting::allObjectsAbortDisarmed(
		ObjectControl& handler) {
	AbstractKinematics::Aborting::allObjectsAbortDisarmed(handler);
	setState(handler, new AbsoluteKinematics::Ready);			
}

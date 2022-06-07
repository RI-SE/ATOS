#include "state.hpp"

AbstractKinematics::RemoteControlled::RemoteControlled() {

}


void AbstractKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
}

void AbstractKinematics::RemoteControlled::onEnter(ObjectControl& handler){
	handler.remoteControlObjects(true);
	handler.startControlSignalSubscriber();
}

void AbstractKinematics::RemoteControlled::onExit(ObjectControl& handler){
	handler.stopControlSignalSubscriber();
	handler.remoteControlObjects(false);
}

/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
	setState(handler, new RelativeKinematics::Ready);
}

void RelativeKinematics::RemoteControlled::onEnter(ObjectControl& handler){
	AbstractKinematics::RemoteControlled::onEnter(handler);
}

void RelativeKinematics::RemoteControlled::onExit(ObjectControl& handler){
	AbstractKinematics::RemoteControlled::onExit(handler);
}

/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::RemoteControlled::onEnter(ObjectControl& handler){
	AbstractKinematics::RemoteControlled::onEnter(handler);
}

void AbsoluteKinematics::RemoteControlled::onExit(ObjectControl& handler){
	AbstractKinematics::RemoteControlled::onExit(handler);
}
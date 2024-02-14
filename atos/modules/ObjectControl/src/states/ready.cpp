/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Ready::Ready() {

}

void AbstractKinematics::Ready::onEnter(
		ObjectControl& handler) {
	handler.startSafetyThread();
}

void AbstractKinematics::Ready::armRequest(
		ObjectControl& handler) {
	if (!handler.areAllObjectsIn(std::set({OBJECT_STATE_DISARMED, OBJECT_STATE_ARMED}))) {
		throw std::invalid_argument("Not all objects in valid state prior to arm");
	}
}

void AbstractKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	handler.disconnectObjects();
}

void AbstractKinematics::Ready::disconnectedFromObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Ready::objectAborting(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Ready::objectAbortDisarmed(
	ObjectControl& /* handler */,
	uint32_t /* id */) {
		// TODO
}

void AbstractKinematics::Ready::settingModificationRequested(
		ObjectControl& /* handler */) {
	// TODO
}
void AbstractKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& /* handler */){
}

void AbstractKinematics::Ready::resetTestObjectsRequest(
		ObjectControl& handler) {
	handler.resetTestObjects();
}

void AbstractKinematics::Ready::reloadObjectSettingsRequest(
		ObjectControl& handler) {
	handler.reloadScenarioTrajectories();
	handler.uploadAllConfigurations();
}

/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */

void RelativeKinematics::Ready::onEnter(
		ObjectControl& handler) {
	AbstractKinematics::Ready::onEnter(handler);
}

void RelativeKinematics::Ready::armRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::armRequest(handler);
	setState(handler, new RelativeKinematics::Armed);
}

void RelativeKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}

void RelativeKinematics::Ready::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Connecting);
}

void RelativeKinematics::Ready::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::objectAborting(handler,id);
	setState(handler, new RelativeKinematics::Aborting);
}


void RelativeKinematics::Ready::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Ready::objectAbortDisarmed(handler,id);
}

void RelativeKinematics::Ready::settingModificationRequested(
		ObjectControl& handler) {
	AbstractKinematics::Ready::settingModificationRequested(handler);
	// TODO call ObjectControl to update OSEM if modified etc.
}

void RelativeKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& handler){
	RCLCPP_WARN(handler.get_logger(), "Relative mode control input conversion not currently supported");
	return;
	//setState(handler, new RelativeKinematics::RemoteControlled);
}

void RelativeKinematics::Ready::resetTestObjectsRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::resetTestObjectsRequest(handler);
}

void RelativeKinematics::Ready::reloadObjectSettingsRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::reloadObjectSettingsRequest(handler);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */

void AbsoluteKinematics::Ready::onEnter(
		ObjectControl& handler) {
	AbstractKinematics::Ready::onEnter(handler);
}

void AbsoluteKinematics::Ready::armRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::armRequest(handler);
	setState(handler, new AbsoluteKinematics::Armed);
}

void AbsoluteKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Ready::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Ready::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::objectAborting(handler,id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Ready::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Ready::objectAbortDisarmed(handler,id);
}

void AbsoluteKinematics::Ready::settingModificationRequested(
		ObjectControl& handler) {
	AbstractKinematics::Ready::settingModificationRequested(handler);
	// TODO call ObjectControl to update OSEM if modified etc.
}

void AbsoluteKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& handler){
	setState(handler, new AbsoluteKinematics::RemoteControlled);
}

void AbsoluteKinematics::Ready::resetTestObjectsRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::resetTestObjectsRequest(handler);
}

void AbsoluteKinematics::Ready::reloadObjectSettingsRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::reloadObjectSettingsRequest(handler);
}

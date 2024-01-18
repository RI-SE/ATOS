/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Aborting::Aborting() {

}

void AbstractKinematics::Aborting::onExit(
		ObjectControl& /* handler */) {
}

void AbstractKinematics::Aborting::allClearRequest(
		ObjectControl&  /* handler */) {
	// TODO
}

void AbstractKinematics::Aborting::connectedToObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Aborting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	RCLCPP_WARN(handler.get_logger(), "Object %d disconnected while CC in aborting state!", id);
}

void AbstractKinematics::Aborting::connectedToLiveObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Aborting::connectedToArmedObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Aborting::objectAborting(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Aborting::objectAbortDisarmed(
	ObjectControl& /* handler */,
	uint32_t /* id */) {
	// TODO
}

void AbstractKinematics::Aborting::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	RCLCPP_WARN(handler.get_logger(), "Object disarmed while expecting abort");
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}

void AbstractKinematics::Aborting::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}


void AbstractKinematics::Aborting::allObjectsAbortDisarmed(
		ObjectControl&  /* handler */) {
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Aborting::allClearRequest(
		ObjectControl& handler) {
	setState(handler, new RelativeKinematics::Clearing);
}

void RelativeKinematics::Aborting::connectedToObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void RelativeKinematics::Aborting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Aborting::disconnectedFromObject(handler, id);
}

void RelativeKinematics::Aborting::connectedToLiveObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void RelativeKinematics::Aborting::connectedToArmedObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
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
	setState(handler, new AbsoluteKinematics::Clearing);
}

void AbsoluteKinematics::Aborting::connectedToObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbsoluteKinematics::Aborting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Aborting::disconnectedFromObject(handler, id);
}

void AbsoluteKinematics::Aborting::connectedToLiveObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
	// TODO
}

void AbsoluteKinematics::Aborting::connectedToArmedObject(
		ObjectControl& /* handler */,
		uint32_t /* id */) {
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

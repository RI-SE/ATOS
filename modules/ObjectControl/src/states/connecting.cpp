/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Connecting::Connecting() {

}

void AbstractKinematics::Connecting::onEnter(
		ObjectControl& handler) {
	handler.beginConnectionAttempt();
}

void AbstractKinematics::Connecting::disconnectRequest(
		ObjectControl& handler) {
	handler.abortConnectionAttempt();
	handler.disconnectObjects();
}

void AbstractKinematics::Connecting::connectRequest(
		ObjectControl& handler) {
	// TODO
}

void AbstractKinematics::Connecting::abortRequest(
		ObjectControl& handler) {
	// TODO
}

void AbstractKinematics::Connecting::connectedToObject(
		ObjectControl& handler,
		uint32_t id) {
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		this->allObjectsConnected(handler);
	}
}

void AbstractKinematics::Connecting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Connecting::connectedToLiveObject(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Connecting::connectedToArmedObject(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Connecting::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Connecting::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Connecting::allObjectsConnected(
		ObjectControl& handler) {
	handler.startListeners();
	handler.notifyObjectsConnected();
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Connecting::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}

void RelativeKinematics::Connecting::connectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::connectRequest(handler);
}

void RelativeKinematics::Connecting::abortRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::abortRequest(handler);
	setState(handler, new RelativeKinematics::Aborting);
}


void RelativeKinematics::Connecting::connectedToObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToObject(handler, id);
}

void RelativeKinematics::Connecting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::disconnectedFromObject(handler, id);
}

void RelativeKinematics::Connecting::connectedToLiveObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToLiveObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::Connecting::connectedToArmedObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToArmedObject(handler, id);
	setState(handler, new RelativeKinematics::Disarming);
}

void RelativeKinematics::Connecting::allObjectsConnected(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::allObjectsConnected(handler);
	setState(handler, new RelativeKinematics::Ready);
}

void RelativeKinematics::Connecting::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::objectArmed(handler, id);
	RelativeKinematics::Connecting::connectedToArmedObject(handler, id);
}

void RelativeKinematics::Connecting::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::objectAborting(handler, id);
	RelativeKinematics::Connecting::connectedToLiveObject(handler, id);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Connecting::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Connecting::connectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::connectRequest(handler);
}

void AbsoluteKinematics::Connecting::abortRequest(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::abortRequest(handler);
	setState(handler, new AbsoluteKinematics::Aborting);
}


void AbsoluteKinematics::Connecting::connectedToObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToObject(handler, id);
}

void AbsoluteKinematics::Connecting::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::disconnectedFromObject(handler, id);
}

void AbsoluteKinematics::Connecting::connectedToLiveObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToLiveObject(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Connecting::connectedToArmedObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::connectedToArmedObject(handler, id);
	setState(handler, new AbsoluteKinematics::Disarming);
}

void AbsoluteKinematics::Connecting::allObjectsConnected(
		ObjectControl& handler) {
	AbstractKinematics::Connecting::allObjectsConnected(handler);
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::Connecting::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::objectArmed(handler, id);
	AbsoluteKinematics::Connecting::connectedToArmedObject(handler, id);
}

void AbsoluteKinematics::Connecting::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Connecting::objectAborting(handler, id);
	AbsoluteKinematics::Connecting::connectedToLiveObject(handler, id);
}

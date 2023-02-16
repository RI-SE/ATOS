/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Disarming::Disarming() {

}

void AbstractKinematics::Disarming::onEnter(
		ObjectControl& handler) {
	handler.disarmObjects();
}

void AbstractKinematics::Disarming::disconnectRequest(
		ObjectControl& handler) {
	handler.disconnectObjects();
}

void AbstractKinematics::Disarming::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::connectedToLiveObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::allObjectsDisarmed(
		ObjectControl&) {
	// TODO
}

void AbstractKinematics::Disarming::objectArmed(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::objectDisarmed(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Disarming::objectAborting(
		ObjectControl&,
		uint32_t) {
	// TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Disarming::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Disarming::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}

void RelativeKinematics::Disarming::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::connectedToLiveObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::connectedToLiveObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::Disarming::allObjectsDisarmed(
		ObjectControl& handler) {
	AbstractKinematics::Disarming::allObjectsDisarmed(handler);
	setState(handler, new RelativeKinematics::Connecting);
}

void RelativeKinematics::Disarming::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectArmed(handler,id);
	RelativeKinematics::Disarming::connectedToArmedObject(handler,id);
}

void RelativeKinematics::Disarming::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		RelativeKinematics::Disarming::allObjectsDisarmed(handler);
	}
}

void RelativeKinematics::Disarming::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectAborting(handler,id);
	RelativeKinematics::Disarming::connectedToLiveObject(handler,id);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Disarming::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Disarming::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Disarming::connectedToObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::connectedToArmedObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::connectedToLiveObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::connectedToLiveObject(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Disarming::allObjectsDisarmed(
		ObjectControl& handler) {
	AbstractKinematics::Disarming::allObjectsDisarmed(handler);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Disarming::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectArmed(handler,id);
	AbsoluteKinematics::Disarming::connectedToArmedObject(handler,id);
}

void AbsoluteKinematics::Disarming::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		AbsoluteKinematics::Disarming::allObjectsDisarmed(handler);
	}
}

void AbsoluteKinematics::Disarming::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Disarming::objectAborting(handler,id);
	AbsoluteKinematics::Disarming::connectedToLiveObject(handler,id);
}

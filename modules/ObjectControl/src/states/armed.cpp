/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Armed::Armed() {

}

void AbstractKinematics::Armed::onEnter(
		ObjectControl& handler) {
	handler.armObjects();
}

void AbstractKinematics::Armed::startRequest(
		ObjectControl& handler) {
	if (!handler.areAllObjectsIn(OBJECT_STATE_ARMED)) {
		throw std::invalid_argument("Start attempted when not all objects were armed");
	}
}

void AbstractKinematics::Armed::disarmRequest(
		ObjectControl&) {
}

void AbstractKinematics::Armed::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
}

void AbstractKinematics::Armed::objectArmed(
		ObjectControl&,
		uint32_t) {
}

void AbstractKinematics::Armed::objectDisarmed(
		ObjectControl&,
		uint32_t) {
}

void AbstractKinematics::Armed::objectAborting(
		ObjectControl&,
		uint32_t) {
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Armed::startRequest(
		ObjectControl& handler) {
	AbstractKinematics::Armed::startRequest(handler);
	setState(handler, new RelativeKinematics::TestLive);
}

void RelativeKinematics::Armed::disarmRequest(
		ObjectControl& handler) {
	AbstractKinematics::Armed::disarmRequest(handler);
	setState(handler, new RelativeKinematics::Disarming);
}

void RelativeKinematics::Armed::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Disarming);
}

void RelativeKinematics::Armed::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectArmed(handler, id);
	// TODO possible to do something here
}

void RelativeKinematics::Armed::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectDisarmed(handler, id);
	setState(handler, new RelativeKinematics::Ready);
}

void RelativeKinematics::Armed::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectAborting(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Armed::startRequest(
		ObjectControl& handler) {
	AbstractKinematics::Armed::startRequest(handler);
	setState(handler, new AbsoluteKinematics::TestLive);
}

void AbsoluteKinematics::Armed::disarmRequest(
		ObjectControl& handler) {
	AbstractKinematics::Armed::disarmRequest(handler);
	setState(handler, new AbsoluteKinematics::Disarming);
}

void AbsoluteKinematics::Armed::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Disarming);
}

void AbsoluteKinematics::Armed::objectArmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectArmed(handler, id);
	// TODO possible to do something here
}

void AbsoluteKinematics::Armed::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectDisarmed(handler, id);
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::Armed::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Armed::objectAborting(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

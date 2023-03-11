/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Clearing::Clearing() {

}

void AbstractKinematics::Clearing::onEnter(
		ObjectControl& handler) {
		handler.allClearObjects();
}

void AbstractKinematics::Clearing::objectArmed(
        ObjectControl&,
        uint32_t) {
    // TODO
}

void AbstractKinematics::Clearing::objectDisarmed(
        ObjectControl&,
        uint32_t) {
    // TODO
}

void AbstractKinematics::Clearing::objectAbortDisarmed(
        ObjectControl&,
        uint32_t) {
    // TODO
}

void AbstractKinematics::Clearing::disconnectedFromObject(
        ObjectControl& handler, 
        uint32_t) {
    // TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Clearing::changeStateIfAllOK(
        ObjectControl& handler
        ) {
    auto disconnected = [](std::pair<uint32_t, std::shared_ptr<TestObject>> obj) {
		return !obj.second->isConnected();
	};
	if (handler.areAllObjectsIn(std::set({OBJECT_STATE_DISARMED, OBJECT_STATE_ARMED}))) {
		setState(handler, new RelativeKinematics::Ready); // All objects are disarmed/armed, so we can go back to ready
	}
	else if (handler.isAnyObject(disconnected)) {
		setState(handler, new RelativeKinematics::Connecting); // Some objects are disconnected, try to reconnect to them
	}
}

void RelativeKinematics::Clearing::objectArmed(
        ObjectControl& handler,
        uint32_t id) {
    changeStateIfAllOK(handler);
}

void RelativeKinematics::Clearing::objectDisarmed(
        ObjectControl& handler,
        uint32_t id) {
    changeStateIfAllOK(handler);
}

void RelativeKinematics::Clearing::objectAbortDisarmed(
        ObjectControl& handler,
        uint32_t id) {
    changeStateIfAllOK(handler);
}

void RelativeKinematics::Clearing::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	RCLCPP_WARN(handler.get_logger(), "Object %d disconnected while CC in clearing state!", id);
    changeStateIfAllOK(handler);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */

void AbsoluteKinematics::Clearing::changeStateIfAllOK(
        ObjectControl& handler
        ) {
    auto readyOrDisconnected = [](std::pair<uint32_t, std::shared_ptr<TestObject>> obj) {
		return obj.second->getState() == OBJECT_STATE_DISARMED ||
                obj.second->getState() == OBJECT_STATE_ARMED ||
                !obj.second->isConnected();
	};
	if (handler.areAllObjectsIn(std::set({OBJECT_STATE_DISARMED, OBJECT_STATE_ARMED}))) {
		setState(handler, new AbsoluteKinematics::Ready); // All objects are disarmed/armed, so we can go back to ready
	}
	else if (handler.areAllObjects(readyOrDisconnected)) {
		setState(handler, new AbsoluteKinematics::Connecting); // Some objects are disconnected, try to reconnect to them
	}
}

void AbsoluteKinematics::Clearing::objectArmed(
        ObjectControl& handler,
        uint32_t id) {
    changeStateIfAllOK(handler);
}

void AbsoluteKinematics::Clearing::objectDisarmed(
        ObjectControl& handler,
        uint32_t id) {
    changeStateIfAllOK(handler);
}

void AbsoluteKinematics::Clearing::objectAbortDisarmed(
    ObjectControl& handler,
    uint32_t id) {
        changeStateIfAllOK(handler);
}

void AbsoluteKinematics::Clearing::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	RCLCPP_WARN(handler.get_logger(), "Object %d disconnected while CC in clearing state!", id);
    changeStateIfAllOK(handler);
}

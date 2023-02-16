/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <stdexcept>

#include "supervisionstate.hpp"

SupervisionState::SupervisionState() {
    // Loopback transitions
    this->transitions.insert( {SupervisionState::READY, SupervisionState::READY} );
    this->transitions.insert( {SupervisionState::VERIFYING_ARM, SupervisionState::VERIFYING_ARM} );
    this->transitions.insert( {SupervisionState::VERIFYING_INIT, SupervisionState::VERIFYING_INIT} );
    this->transitions.insert( {SupervisionState::RUNNING, SupervisionState::RUNNING} );

    // Transitions to ready
    this->transitions.insert( {SupervisionState::VERIFYING_ARM, SupervisionState::READY} );
    this->transitions.insert( {SupervisionState::VERIFYING_INIT, SupervisionState::READY} );
    this->transitions.insert( {SupervisionState::RUNNING, SupervisionState::READY} );

    // Transitions to verifying init
    this->transitions.insert( {SupervisionState::READY, SupervisionState::VERIFYING_INIT} );

    // Transitions to verifying arm
    this->transitions.insert( {SupervisionState::READY, SupervisionState::VERIFYING_ARM} );

    // Transitions to verifying running
    this->transitions.insert( {SupervisionState::READY, SupervisionState::RUNNING} );

}

void SupervisionState::set(SupervisionState::State newState) {
    for (const std::pair<SupervisionState::State, SupervisionState::State> &transition : transitions)
    {
        if (transition.first == currentState && transition.second == newState) {
            currentState = newState;
            return;
        }
    }
    currentState = ERROR;
}

std::string SupervisionState::toString(State state) {
    switch (state) {
    case READY:
        return "READY";
    case VERIFYING_INIT:
        return "VERIFYING INIT";
    case VERIFYING_ARM:
        return "VERIFYING ARM";
    case RUNNING:
        return "RUNNING";
    case ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

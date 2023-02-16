/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef SUPERVISIONSTATE_H
#define SUPERVISIONSTATE_H

#include <set>
#include <iostream>

class SupervisionState {
public:
    typedef enum {
        READY,
        VERIFYING_INIT,
        VERIFYING_ARM,
        RUNNING,
        ERROR
    } State;
    void set(State newState);
    State get() const { return this->currentState; }
    static std::string toString(State state);

    SupervisionState();
private:
    State currentState = READY;
    std::set<std::pair<State, State>> transitions;
};

#endif

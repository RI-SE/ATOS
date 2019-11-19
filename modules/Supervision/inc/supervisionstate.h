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
        RUNNING
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

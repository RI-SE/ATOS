#include "sml.hpp"
#include "objectcontrol.hpp"

#pragma once

// State transitions
class SmImpl {
public:
    // Events
    struct initializeRequest {};
    struct ev_stop {};

    // Actions
    constexpr static auto clearScenarioAction = [](ObjectControl* handler) { handler->clearScenario(); };
    constexpr static auto ac_stop  = [](ObjectControl* handler) { handler->loadScenario(); };

    auto operator()() const noexcept {
        using namespace boost::sml;
        return make_transition_table(
                 *state<AbstractKinematics::Idle>       + event<initializeRequest>    / clearScenarioAction    =    state<AbstractKinematics::Initialized>
                , "Driving"_s    + event<ev_stop>     / ac_stop     =    "Idle"_s
        );
    }
};
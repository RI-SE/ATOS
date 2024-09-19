#include "sml.hpp"
#include "objectcontrol.hpp"

#pragma once

// State transitions
class SmImpl {
public:
    // Events
    struct initializeRequest {};
    struct ev_stop {};

    // Guards
    constexpr static auto guard = [](ObjectControl* handler) { return true; };

    // Actions
    constexpr static auto clearScenarioAction = [](ObjectControl* handler) { handler->clearScenario(); };
    constexpr static auto ac_stop  = [](ObjectControl* handler) { handler->loadScenario(); };

    auto operator()() const noexcept {
        using namespace boost::sml;
        return make_transition_table(
                 *"idle"_s + event<initializeRequest> [ guard ] / clearScenarioAction = "initialized"_s
                , "Driving"_s    + event<ev_stop>     / ac_stop     =    "Idle"_s
        );
    }
};
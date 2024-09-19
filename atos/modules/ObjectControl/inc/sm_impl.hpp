#include "sml.hpp"
#include "objectcontrol.hpp"

#pragma once

// State transitions
class SmImpl {
public:
    // Events
    struct InitializeRequest {};

    // Guards
    constexpr static auto guard = [](ObjectControl* handler) { return true; };

    // Actions
    constexpr static auto clearScenarioAction = [](ObjectControl* handler) { handler->clearScenario(); };

    auto operator()() const noexcept {
        using namespace boost::sml;
        return make_transition_table(
                 *state<AbstractKinematics::Idle> + event<InitializeRequest> [ guard ] / clearScenarioAction = state<AbstractKinematics::Initialized>
        );
    }
};
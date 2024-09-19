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

    constexpr static auto initializeRequest = [](ObjectControl* handler) { handler->initializeRequest(); };

    void initializeRequest(
		ObjectControl& handler) {
	RCLCPP_INFO(handler.get_logger(), "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
	bool successful = handler.loadScenario(); // Reload objects on each initialize request. 
	if (!successful) {
		RCLCPP_ERROR(handler.get_logger(), "Failed to load scenario");
		JournalRecordData(JOURNAL_RECORD_EVENT, "INIT failed");
		return;
	}
	try {
		auto anchorID = handler.getAnchorObjectID();
		handler.transformScenarioRelativeTo(anchorID);
		handler.controlMode = ObjectControl::RELATIVE_KINEMATICS;
		setState(handler, new RelativeKinematics::Initialized);
		RCLCPP_INFO(handler.get_logger(), "Relative control mode enabled");
	} catch (std::invalid_argument&) {
		handler.controlMode = ObjectControl::ABSOLUTE_KINEMATICS;
		setState(handler, new AbsoluteKinematics::Initialized);
		RCLCPP_INFO(handler.get_logger(), "Absolute control mode enabled");
	}
}
    constexpr static auto ac_stop  = [](ObjectControl* handler) { handler->loadScenario(); };

    auto operator()() const noexcept {
        using namespace boost::sml;
        return make_transition_table(
                 *state<AbstractKinematics::Idle> + event<initializeRequest> [ guard ] / clearScenarioAction = state<AbstractKinematics::Ready>
                 , state<AbstractKinematics::Read>
        );
    }
};
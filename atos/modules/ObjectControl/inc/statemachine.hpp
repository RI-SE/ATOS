/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <cstdint>
#include <type_traits>

#include "roschannels/commandchannels.hpp"
#include "roschannels/objstatechangechannel.hpp"
#include "roschannels/statechange.hpp"
#include "sml.hpp"
#include "util.h"

class ObjectControl;

namespace state_machine {
namespace events {

struct Initialize {};
struct Connect {};
struct Disconnect {};
struct DisconnectedFromObject {
	uint32_t id{};
};
struct Ready {};
struct Reset {};
struct Reload {};
struct Arm {};
struct Disarm {};
struct TestLive {};
struct RemoteControl {};
struct Clear {};
struct Abort {};
struct Start {};
struct StartObject {
	uint32_t id{};
	std::chrono::system_clock::time_point startTime{};
};
struct Done {};

} // namespace events

static constexpr auto Idle			= boost::sml::state<struct Idle>;
static constexpr auto Initialized	= boost::sml::state<struct Initialized>;
static constexpr auto Connecting	= boost::sml::state<struct Connecting>;
static constexpr auto Disconnecting = boost::sml::state<struct Disconnecting>;
static constexpr auto Ready			= boost::sml::state<struct Ready>;
static constexpr auto Armed			= boost::sml::state<struct Armed>;
static constexpr auto Disarming		= boost::sml::state<struct Disarming>;
static constexpr auto TestLive		= boost::sml::state<struct TestLive>;
static constexpr auto Aborting		= boost::sml::state<struct Aborting>;
static constexpr auto Clearing		= boost::sml::state<struct Clearing>;
static constexpr auto Done			= boost::sml::state<struct Done>;
static constexpr auto RemoteControl = boost::sml::state<struct RemoteControl>;

// clang-format off
struct idle_on_entry             { void operator()(ObjectControl* oc) const; };
struct idle_to_init_guard        { bool operator()(ObjectControl* oc) const; };
struct idle_to_init_action       { void operator()(ObjectControl* oc) const; };
struct init_to_connecting_action { void operator()(ObjectControl* oc) const; };
struct init_to_connecting_guard  { bool operator()(ObjectControl* oc) const; };
struct init_to_idle_action       { void operator()(ObjectControl* oc) const; };
struct connecting_on_entry       { void operator()(ObjectControl* oc) const; };
struct connecting_to_idle_action { void operator()(ObjectControl* oc) const; };
struct clearing_on_entry         { void operator()(ObjectControl* oc) const; };
struct ready_on_entry            { void operator()(ObjectControl* oc) const; };
struct ready_reset               { void operator()(ObjectControl* oc) const; };
struct ready_reload              { void operator()(ObjectControl* oc) const; };
struct remote_control_on_entry   { void operator()(ObjectControl* oc) const; };
struct remote_control_on_exit    { void operator()(ObjectControl* oc) const; };
struct disarming_on_entry        { void operator()(ObjectControl* oc) const; };
struct done_on_entry             { void operator()(ObjectControl* oc) const; };
struct armed_on_enter            { void operator()(ObjectControl* oc) const; };
struct armed_to_testlive_guard   { bool operator()(ObjectControl* oc) const; };
struct test_live_start_object    { void operator()(const events::StartObject& event, ObjectControl* oc) const; };
struct disconnected_from_object  { void operator()(const events::DisconnectedFromObject& event, ObjectControl* oc) const; };
// clang-format on

template<class T>
OBCState_t asNumber() {
	if constexpr (std::is_same_v<typename T::type, decltype(Idle)::type>) {
		return OBC_STATE_IDLE;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Initialized)::type>) {
		return OBC_STATE_INITIALIZED;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Connecting)::type>) {
		return OBC_STATE_CONNECTED;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Ready)::type>) {
		return OBC_STATE_CONNECTED;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Armed)::type>) {
		return OBC_STATE_ARMED;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Disarming)::type>) {
		return OBC_STATE_DISARMING;
	} else if constexpr (std::is_same_v<typename T::type, decltype(TestLive)::type>) {
		return OBC_STATE_RUNNING;
	} else if constexpr (std::is_same_v<typename T::type, decltype(RemoteControl)::type>) {
		return OBC_STATE_REMOTECTRL;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Aborting)::type>) {
		return OBC_STATE_ABORTING;
	} else if constexpr (std::is_same_v<typename T::type, decltype(Clearing)::type>) {
		return OBC_STATE_CLEARING;
	} else {
		return OBC_STATE_UNDEFINED;
	}
}

class Logger {
public:
	Logger(rclcpp::Logger l, ROSChannels::StateChange::Pub sc, ROSChannels::Failure::Pub f) :
	  logger_{l},
	  stateChangePub_{sc},
	  failurePub_{f} {}

	template<class SM, class TEvent>
	void log_process_event(const TEvent&) {
		RCLCPP_DEBUG(logger_,
					 "[%s][process_event] %s",
					 boost::sml::aux::get_type_name<SM>(),
					 boost::sml::aux::get_type_name<TEvent>());
	}

	template<class SM, class TGuard, class TEvent>
	void log_guard(const TGuard&, const TEvent&, bool result) {
		RCLCPP_DEBUG(logger_,
					 "[%s][guard] %s %s %s",
					 boost::sml::aux::get_type_name<SM>(),
					 boost::sml::aux::get_type_name<TGuard>(),
					 boost::sml::aux::get_type_name<TEvent>(),
					 (result ? "[OK]" : "[Reject]"));
	}

	template<class SM, class TAction, class TEvent>
	void log_action(const TAction&, const TEvent&) {
		RCLCPP_DEBUG(logger_,
					 "[%s][action] %s %s",
					 boost::sml::aux::get_type_name<SM>(),
					 boost::sml::aux::get_type_name<TAction>(),
					 boost::sml::aux::get_type_name<TEvent>());
	}

	template<class SM, class TSrcState, class TDstState>
	void log_state_change(const TSrcState& src, const TDstState& dst) {
		RCLCPP_DEBUG(
		  logger_, "[%s][transition] %s -> %s", boost::sml::aux::get_type_name<SM>(), src.c_str(), dst.c_str());
		auto stateChangeMsg			 = atos_interfaces::msg::StateChange();
		stateChangeMsg.prev_state	 = asNumber<TSrcState>();
		stateChangeMsg.current_state = asNumber<TDstState>();
		stateChangePub_.publish(stateChangeMsg);
	}

private:
	rclcpp::Logger logger_;
	ROSChannels::StateChange::Pub stateChangePub_;
	ROSChannels::Failure::Pub failurePub_;
};

class StateMachine {
public:
	auto operator()() const noexcept {
		namespace sml = boost::sml;
		return sml::make_transition_table(
		  *Idle + sml::on_entry<sml::_> / idle_on_entry(),
		  Idle + sml::event<events::Initialize>[idle_to_init_guard()] / idle_to_init_action() = Initialized,

		  Initialized + sml::event<events::Disconnect> / init_to_idle_action() = Idle,
		  Initialized + sml::event<events::Connect>[init_to_connecting_guard()] / init_to_connecting_action() =
			Connecting,

		  Connecting + sml::on_entry<sml::_> / connecting_on_entry(),
		  Connecting + sml::event<events::Abort>									= Aborting,
		  Connecting + sml::event<events::Disarm>									= Disarming,
		  Connecting + sml::event<events::Disconnect> / connecting_to_idle_action() = Idle,
		  Connecting + sml::event<events::Ready>									= Ready,

		  Ready + sml::on_entry<sml::_> / ready_on_entry(),
		  Ready + sml::event<events::Abort>												  = Aborting,
		  Ready + sml::event<events::Arm>												  = Armed,
		  Ready + sml::event<events::Disconnect>										  = Idle,
		  Ready + sml::event<events::DisconnectedFromObject> / disconnected_from_object() = Connecting,
		  Ready + sml::event<events::RemoteControl>										  = RemoteControl,
		  Ready + sml::event<events::Reset> / ready_reset(),
		  Ready + sml::event<events::Reload> / ready_reload(),

		  Armed + sml::on_entry<sml::_> / armed_on_enter(),
		  Armed + sml::event<events::Start>[armed_to_testlive_guard()] = TestLive,
		  Armed + sml::event<events::Disarm>						   = Disarming,

		  Disarming + sml::on_entry<sml::_> / disarming_on_entry(),
		  Disarming + sml::event<events::Abort>		 = Aborting,
		  Disarming + sml::event<events::Connect>	 = Connecting,
		  Disarming + sml::event<events::Disconnect> = Idle,
		  Disarming + sml::event<events::Ready>		 = Ready,

		  TestLive + sml::event<events::Abort> = Aborting,
		  TestLive + sml::event<events::Done>  = Done,
		  TestLive + sml::event<events::StartObject> / test_live_start_object(),

		  Done + sml::on_entry<sml::_> / done_on_entry(),
		  Done + sml::event<events::Ready> = Ready,

		  Aborting + sml::event<events::Clear> = Clearing,

		  Clearing + sml::on_entry<sml::_> / clearing_on_entry(),
		  Clearing + sml::event<events::Ready> = Ready,

		  RemoteControl + sml::on_entry<sml::_> / remote_control_on_entry(),
		  RemoteControl + sml::on_exit<sml::_> / remote_control_on_exit(),
		  RemoteControl + sml::event<events::Ready> = Ready);
	}
};

} // namespace state_machine

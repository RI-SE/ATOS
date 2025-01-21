#include "sml.hpp"
#include "objectcontrol.hpp"
#include <chrono>

#pragma once

// State transitions
class SmImpl {
public:
    // States
    constexpr static auto idle = boost::sml::state<class Idle>;
    constexpr static auto initialized = boost::sml::state<class Initialized>;
    constexpr static auto connecting = boost::sml::state<class Connecting>; // Is this really a state? could be a transition, if we wait for all objects to connect
    constexpr static auto armed = boost::sml::state<class Armed>;
    constexpr static auto disarming = boost::sml::state<class Disarming>; // Same with this one
    constexpr static auto aborting = boost::sml::state<class Aborting>; // Same with this one
    constexpr static auto ready = boost::sml::state<class Ready>;
    constexpr static auto testlive = boost::sml::state<class TestLive>;

    // Events
    struct InitializeRequest {};
    struct ConnectRequest {};
    struct DisconnectRequest {};
    struct ArmRequest {};
    struct DisarmRequest {};
    struct StartRequest {};
    struct AbortRequest {};
    struct StartObjectRequest {
        uint32_t id; 
        std::chrono::system_clock::time_point startTime;
    };
    struct ConnectedToObject {};
    struct ConnectedToLiveObject {};
    struct DisconnectedFromObject {};
    struct ObjectDisarmed {};
    struct AllObjectsConnected {};
    struct AllObjectsDisarmed {};
    struct ObjectAborting {};

    // Guards
    struct scenarioLoaded {
        auto operator()(ObjectControl* handler) {
            return handler->loadScenario();
        }
    } scenarioLoaded;

    struct allObjectsConnected{
        auto operator()(ObjectControl* handler) {
            handler->areAllObjectsIn(OBJECT_STATE_DISARMED);
        };
    } allObjectsConnected;

    struct anyObjectInArmedState{
        auto operator()(ObjectControl* handler) {
            handler->isAnyObjectIn(OBJECT_STATE_ARMED);
        };
    } anyObjectInArmedState;

    struct allObjectsInArmedState{
        auto operator()(ObjectControl* handler) {
            handler->areAllObjectsIn(OBJECT_STATE_ARMED);
        };
    } allObjectsInArmedState;

    struct anyObjectInRunningState{
        auto operator()(ObjectControl* handler) {
            handler->isAnyObjectIn(OBJECT_STATE_RUNNING);
        };
    } anyObjectInRunningState;

    struct allObjectsDisarmedOrDisconnected{
        auto operator()(ObjectControl* handler) {
            static auto disarmedOrDisconnected = [](const std::shared_ptr<TestObject> obj) {
                return obj->getState() == OBJECT_STATE_DISARMED || !obj->isConnected();
            };
            handler->areAllObjects(disarmedOrDisconnected);
        };
    } allObjectsDisarmedOrDisconnected;

    struct allObjectsDisarmed{
        auto operator()(ObjectControl* handler) {
            handler->areAllObjectsIn(OBJECT_STATE_DISARMED);
        };
    } allObjectsDisarmed;

    // Actions
    struct setKinematicsMode {
        auto operator()(ObjectControl* handler) { 	
            //JournalRecordData(JournalRecordType::JOURNAL_RECORD_EVENT, "INIT successful");
            try {
                auto anchorID = handler->getAnchorObjectID();
                handler->transformScenarioRelativeTo(anchorID);
                handler->controlMode = ObjectControl::RELATIVE_KINEMATICS;
                RCLCPP_INFO(handler->get_logger(), "Relative control mode enabled");
            } catch (std::invalid_argument&) {
                handler->controlMode = ObjectControl::ABSOLUTE_KINEMATICS;
                RCLCPP_INFO(handler->get_logger(), "Absolute control mode enabled");
            }
        }
    } setKinematicsMode;

    struct connectToTestObjects{
        auto operator()(ObjectControl* handler) {
            //RCLCPP_INFO(handler->get_logger(), "Handling connect request");
            //JournalRecordData(JOURNAL_RECORD_EVENT, "CONNECT received");
            handler->beginConnectionAttempt();
        };
    } connectToTestObjects;

    struct clearScenario{
        auto operator()(ObjectControl* handler) {
            handler->clearScenario();
        };
    } clearScenario;

    struct disconnectFromTestObjects{
        auto operator()(ObjectControl* handler) {
            //JournalRecordData(JOURNAL_RECORD_EVENT, "DISCONNECT received");
            handler->abortConnectionAttempt();
            handler->disconnectObjects();
        };
    } disconnectFromTestObjects;

    struct armObjects{
        auto operator()(ObjectControl* handler) {
            //JournalRecordData(JOURNAL_RECORD_EVENT, "ARM received");
            handler->armObjects();
        };
    } armObjects;

    struct startListeners{
        auto operator()(ObjectControl* handler) {
            handler->startListeners();
            handler->notifyObjectsConnected();
        };
    } startListeners;

    struct disarmObjects{
        auto operator()(ObjectControl* handler) {
            //JournalRecordData(JOURNAL_RECORD_EVENT, "DISARM received");
            handler->disarmObjects();
        };
    } disarmObjects;

    struct startObjectRequest{
        auto operator()(ObjectControl* handler, StartObjectRequest const& request) {
            handler->startObject(request.id, request.startTime);
        };
    } startObjectRequest;

    auto operator()() const noexcept {
        using namespace boost::sml;
        return make_transition_table(
                // Idle states
                *idle + event<InitializeRequest> [scenarioLoaded] / setKinematicsMode = initialized,
                *idle + event<InitializeRequest> [!scenarioLoaded] = idle,
                
                // Initialized can go back to idle or connecting
                initialized + event<ConnectRequest> = connecting,
                initialized + event<DisconnectRequest> / clearScenario = idle,

                // Try to connect
                connecting + event<DisconnectRequest> / disconnectFromTestObjects = idle,
                connecting + event<ConnectedToObject> [anyObjectInArmedState] = disarming,
                connecting + event<ConnectedToObject> [anyObjectInRunningState] = aborting,
                connecting + event<ConnectedToObject> [!allObjectsConnected] = connecting,
                connecting + event<ConnectedToObject> [allObjectsConnected] / armObjects = armed,
                connecting + event<AllObjectsConnected> / startListeners = ready,

                // Try to disarm
                disarming + event<DisconnectRequest> / disconnectFromTestObjects = idle,
                disarming + event<ObjectDisarmed> [allObjectsDisarmedOrDisconnected] = ready,
                disarming + event<ObjectDisarmed> [!allObjectsDisarmedOrDisconnected] = disarming,
                disarming + event<AllObjectsDisarmed> = ready,

                // Armed state (some transitions missing here..?)
                armed + event<StartRequest> [allObjectsInArmedState] = testlive,
                armed + event<DisarmRequest> = disarming,
                armed + event<ObjectDisarmed> = ready,
                armed + event<DisconnectedFromObject> = disarming,

                // TestLive state
                testlive + event<AbortRequest> = aborting,
                testlive + event<StartObjectRequest> / startObjectRequest = testlive,

                // Abort when object requests it, when the operator request it, or when something goes wrong
                connecting + event<ObjectAborting> = aborting,
                connecting + event<ConnectedToLiveObject> = aborting,
                connecting + event<AbortRequest> = aborting,
                disarming + event<ObjectAborting> = aborting,
                disarming + event<ConnectedToLiveObject> = aborting,
                disarming + event<AbortRequest> = aborting,
                armed + event<AbortRequest> = aborting,
                armed + event<ObjectAborting> = aborting,
                armed + event<ConnectedToLiveObject> = aborting,

                // on_entry functions for states
                idle + on_entry<_> / clearScenario,
                connecting + on_entry<_> / connectToTestObjects,
                armed + on_entry<_> / armObjects,
                disarming + on_entry<_> / disarmObjects
        );
    }
};
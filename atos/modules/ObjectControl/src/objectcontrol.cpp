/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>
#include <dirent.h>
#include <exception>

#include "sm_impl.hpp"
#include "state.hpp"
#include "util.h"
#include "journal.hpp"

#include "objectcontrol.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace ATOS;
namespace sml = boost::sml;

class ObjectControl::Sm : public sml::sm<SmImpl,sml::thread_safe<std::mutex>> {
	public: 
		explicit Sm(ObjectControl* handler)
        	: sml::sm<SmImpl,sml::thread_safe<std::mutex>>(static_cast<ObjectControl*>(handler)) {}
};

ObjectControl::ObjectControl(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
	: Module(ObjectControl::moduleName),
	sm(std::make_unique<Sm>(this)),
	exec(exec),
	scnInitSub(*this, std::bind(&ObjectControl::onInitMessage, this, _1)),
	scnStartSub(*this, std::bind(&ObjectControl::onStartMessage, this, _1)),
	objectStartSub(*this, std::bind(&ObjectControl::onStartObjectMessage, this, _1)),
	scnArmSub(*this, std::bind(&ObjectControl::onArmMessage, this, _1)),
	scnDisarmSub(*this, std::bind(&ObjectControl::onDisarmMessage, this, _1)),
	scnStopSub(*this, std::bind(&ObjectControl::onStopMessage, this, _1)),
	scnAbortSub(*this, std::bind(&ObjectControl::onAbortMessage, this, _1)),
	scnAllClearSub(*this, std::bind(&ObjectControl::onAllClearMessage, this, _1)),
	scnConnectSub(*this, std::bind(&ObjectControl::onConnectMessage, this, _1)),
	scnDisconnectSub(*this, std::bind(&ObjectControl::onDisconnectMessage, this, _1)),
	scnRemoteControlEnableSub(*this, std::bind(&ObjectControl::onRemoteControlEnableMessage, this, _1)),
	scnRemoteControlDisableSub(*this, std::bind(&ObjectControl::onRemoteControlDisableMessage, this, _1)),
	getStatusSub(*this, std::bind(&ObjectControl::onGetStatusMessage, this, _1)),	
	objectStateChangeSub(*this, std::bind(&ObjectControl::onObjectStateChangeMessage, this, _1)),
	scnResetTestObjectsSub(*this, std::bind(&ObjectControl::onResetTestObjectsMessage, this, _1)),
	scnReloadObjectSettingsSub(*this, std::bind(&ObjectControl::onReloadObjectSettingsMessage, this, _1)),
	failurePub(*this),
	scnAbortPub(*this),
	objectsConnectedPub(*this),
	connectedObjectIdsPub(*this),
	stateChangePub(*this)
{
	id_client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	traj_client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);;
	ip_client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	origin_client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	this->declare_parameter("max_missing_heartbeats", 100);
	objectsConnectedTimer = create_wall_timer(1000ms, std::bind(&ObjectControl::publishObjectIds, this));
	idClient = create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds, rmw_qos_profile_services_default, id_client_cb_group_);
	originClient = create_client<atos_interfaces::srv::GetTestOrigin>(ServiceNames::getTestOrigin, rmw_qos_profile_services_default, origin_client_cb_group_);
	trajectoryClient = create_client<atos_interfaces::srv::GetObjectTrajectory>(ServiceNames::getObjectTrajectory, rmw_qos_profile_services_default, traj_client_cb_group_);
	ipClient = create_client<atos_interfaces::srv::GetObjectIp>(ServiceNames::getObjectIp, rmw_qos_profile_services_default, ip_client_cb_group_);
	returnTrajectoryClient = create_client<atos_interfaces::srv::GetObjectReturnTrajectory>(ServiceNames::getObjectReturnTrajectory);
	stateService = create_service<atos_interfaces::srv::GetObjectControlState>(ServiceNames::getObjectControlState,
		std::bind(&ObjectControl::onRequestState, this, _1, _2));

	// Set the initial state
	this->state = static_cast<ObjectControlState*>(new AbstractKinematics::Idle);
	// Create test journal
	if (JournalInit(get_name(), get_logger()) == -1) {
		RCLCPP_ERROR(get_logger(), "Unable to create test journal");
	}
};

ObjectControl::~ObjectControl() {
	delete state;
}

void ObjectControl::onRequestState(
		const std::shared_ptr<atos_interfaces::srv::GetObjectControlState::Request>,
		std::shared_ptr<atos_interfaces::srv::GetObjectControlState::Response> res) {
	try {
		if (!this->state) {
			throw std::runtime_error("No state set");
		}
		res->state = this->state->asNumber();
		res->success = true;
	}
	catch (std::exception& e) {
		RCLCPP_ERROR(get_logger(), "Failed to get state: %s", e.what());
		res->state = -1;
		res->success = false;
	}
}

void ObjectControl::handleActionConfigurationCommand(
		const TestScenarioCommandAction& /* action */) {
	this->state->settingModificationRequested(*this);
	// TODO: Implement
}

void ObjectControl::handleExecuteActionCommand(
		const uint16_t &actionID,
		const std::chrono::system_clock::time_point &when) {
	this->state->actionExecutionRequested(*this);
	auto delayedExecutor = [&](){
		using namespace std::chrono;
		RCLCPP_DEBUG(get_logger(), "Executing action %u in %ld ms", actionID,
				   duration_cast<milliseconds>(when - system_clock::now()).count());
		std::this_thread::sleep_until(when);
		RCLCPP_INFO(get_logger(), "Executing action %u", actionID);
		this->storedActions[actionID]();
	};
	auto thd = std::thread(delayedExecutor);
	thd.detach();
}

void ObjectControl::onInitMessage(const Init::message_type::SharedPtr){
	COMMAND cmd = COMM_INIT;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->initializeRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Init::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	// Don't publish until we are actually initialized
	RCLCPP_INFO(get_logger(), "State change from %d to %d", stateChangeMsg.prev_state, stateChangeMsg.current_state);
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onConnectMessage(const Connect::message_type::SharedPtr){	
	COMMAND cmd = COMM_CONNECT;
	// TO DO: Fix publishing correct data to stateChangeMsg
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->connectRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Connect::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onArmMessage(const Arm::message_type::SharedPtr){	
	COMMAND cmd = COMM_ARM;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->armRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Arm::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onDisarmMessage(const Disarm::message_type::SharedPtr){	
	COMMAND cmd = COMM_DISARM;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->disarmRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Disarm::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onStartMessage(const Start::message_type::SharedPtr){	
	COMMAND cmd = COMM_STRT;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->startRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Start::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onStartObjectMessage(const StartObject::message_type::SharedPtr strtObj){
	using namespace std::chrono;
	COMMAND cmd = COMM_START_OBJECT;
	auto f_try = [&]() { this->state->startObjectRequest(*this, strtObj->id, system_clock::time_point{seconds{strtObj->stamp.sec} + nanoseconds{strtObj->stamp.nanosec}});};
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, StartObject::topicName, get_logger());
}

void ObjectControl::onDisconnectMessage(const Disconnect::message_type::SharedPtr){	
	COMMAND cmd = COMM_DISCONNECT;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->disconnectRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Disconnect::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onStopMessage(const Stop::message_type::SharedPtr){
	COMMAND cmd = COMM_STOP;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->stopRequest(*this); };
	auto f_catch = [&]() {
			failurePub.publish(msgCtr1<Failure::message_type>(cmd));
			scnAbortPub.publish(Abort::message_type());
	};
	this->tryHandleMessage(f_try,f_catch, Stop::topicName, get_logger());	
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

/**
 * @brief Resets the test objects to the scenario starting positions
 * 
 * @param msg Trigger message for resetting test objects (std_msgs/msg/Empty)
*/
void ObjectControl::onResetTestObjectsMessage(const ResetTestObjects::message_type::SharedPtr) {
	COMMAND cmd = COMM_BACKTOSTART_CALL;
	auto f_try = [&]() { this->state->resetTestObjectsRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, ResetTestObjects::topicName, get_logger());
}

/**
 * @brief Reloads the object settings from the configuration file
 * 
 * @param msg Trigger message for reloading object settings (std_msgs/msg/Empty)
*/
void ObjectControl::onReloadObjectSettingsMessage(const ReloadObjectSettings::message_type::SharedPtr) {
	COMMAND cmd = COMM_OSEM;
	auto f_try = [&]() { this->state->reloadObjectSettingsRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, ReloadObjectSettings::topicName, get_logger());
}

void ObjectControl::onAbortMessage(const Abort::message_type::SharedPtr){
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
  publishScenarioInfoToJournal(); // TODO: This should be moved to a state that occurs right after a test is finished
	// Any exceptions here should crash the program
	this->state->abortRequest(*this);
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onAllClearMessage(const AllClear::message_type::SharedPtr){
	COMMAND cmd = COMM_ABORT_DONE;
	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = this->state->asNumber();
	auto f_try = [&]() { this->state->allClearRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, AllClear::topicName, get_logger());
	stateChangeMsg.current_state = this->state->asNumber();
	stateChangePub.publish(stateChangeMsg);
}

void ObjectControl::onRemoteControlEnableMessage(const RemoteControlEnable::message_type::SharedPtr){
	COMMAND cmd = COMM_REMOTECTRL_ENABLE;
	auto f_try = [&]() { this->state->enableRemoteControlRequest(*this); };
	auto f_catch = [&]() {
			failurePub.publish(msgCtr1<Failure::message_type>(cmd));
	};
	this->tryHandleMessage(f_try,f_catch, RemoteControlEnable::topicName, get_logger());	
}

void ObjectControl::onRemoteControlDisableMessage(const RemoteControlDisable::message_type::SharedPtr){
	COMMAND cmd = COMM_REMOTECTRL_DISABLE;
	auto f_try = [&]() { this->state->disableRemoteControlRequest(*this); };
	auto f_catch = [&]() {
			failurePub.publish(msgCtr1<Failure::message_type>(cmd));
	};
	this->tryHandleMessage(f_try,f_catch, RemoteControlDisable::topicName, get_logger());	
}

void ObjectControl::onControlSignalMessage(const ControlSignal::message_type::SharedPtr csp){
	try{
		objects.at(csp->atos_header.object_id)->sendControlSignal(csp);
	}
	catch(const std::exception& e){
		RCLCPP_ERROR(get_logger(), "Failed to translate/send Control Signal Percentage: %s", e.what());
	}
}

void ObjectControl::onPathMessage(const Path::message_type::SharedPtr trajlet,uint32_t id){
	objects.at(id)->setLastReceivedPath(trajlet);
}

bool ObjectControl::loadScenario() {
	this->isResetting = false;
	this->clearScenario();
	RCLCPP_INFO(get_logger(), "Loading scenario");
	std::promise<bool> scenarioLoaded;
	auto idsCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedFuture future) {
		auto idResponse = future.get();
		bool successful = true;
		RCLCPP_INFO(get_logger(), "Received %lu configured object ids", idResponse->ids.size());

		for (const auto id : idResponse->ids) {
			auto object = std::make_shared<TestObject>(id);
			exec->add_node(object);
			objects.emplace(id, object);
			objects.at(id)->setTransmitterID(id);

			std::promise<bool> trajLoaded;
			auto trajectoryCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedFuture future) {
				if (!future.get()->success) {
					RCLCPP_ERROR(get_logger(), "Get trajectory service call failed for object ID %u", id);
					trajLoaded.set_value(false);
					return;
				}
				ATOS::Trajectory traj(get_logger());
				traj.initializeFromCartesianTrajectory(future.get()->trajectory);
				objects.at(id)->setTrajectory(traj);
				trajLoaded.set_value(true);
				RCLCPP_INFO(get_logger(), "Loaded trajectory for object ID %u with %lu points", id, traj.size());
			};
			auto trajRequest = std::make_shared<atos_interfaces::srv::GetObjectTrajectory::Request>();
			trajRequest->id = id;
			trajectoryClient->async_send_request(trajRequest, trajectoryCallback);

			std::promise<bool> ipLoaded;
			auto ipCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetObjectIp>::SharedFuture future) {
				if (!future.get()->success) {
					RCLCPP_ERROR(get_logger(), "Get IP service call failed for object ID %u", id);
					ipLoaded.set_value(false);
					return;
				}
				// Resolve the hostname or numerical IP address to an in_addr_t value
				addrinfo* result;
				addrinfo hints = {};
				hints.ai_family = AF_INET; // Use AF_INET6 for IPv6
				hints.ai_socktype = SOCK_STREAM;

				int status = getaddrinfo(future.get()->ip.c_str(), nullptr, &hints, &result);
				if (status != 0) {
					RCLCPP_ERROR(get_logger(), "Failed to resolve address for object ID %u: %s", id, gai_strerror(status));
					ipLoaded.set_value(false);
					return;
				}

				in_addr_t ip = ((sockaddr_in*)result->ai_addr)->sin_addr.s_addr;
				freeaddrinfo(result);

				// Convert binary IP to string for logging
				char ip_str[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, &ip, ip_str, INET_ADDRSTRLEN);

				RCLCPP_INFO(get_logger(), "Got ip %s for object ID %u", ip_str, id);
				objects.at(id)->setObjectIP(ip);
				ipLoaded.set_value(true);
			};

			auto ipRequest = std::make_shared<atos_interfaces::srv::GetObjectIp::Request>();
			ipRequest->id = id;
			ipClient->async_send_request(ipRequest, ipCallback);


			// Get test origin
			std::promise<bool> originLoaded;
			auto originCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetTestOrigin>::SharedFuture future) {
				if (!future.get()->success) {
					RCLCPP_ERROR(get_logger(), "Get origin service call failed for object ID %u", id);
					originLoaded.set_value(false);
					return;
				}
				auto origin = future.get();
				objects.at(id)->setOrigin({
					origin->origin.position.latitude,
					origin->origin.position.longitude,
					origin->origin.position.altitude,
					true,true,true});
				RCLCPP_INFO(get_logger(), "Got origin for object ID %u: (%.6f, %.6f, %.3f)", id,
					origin->origin.position.latitude,
					origin->origin.position.longitude,
					origin->origin.position.altitude);
					originLoaded.set_value(true);
			};
			auto requestOrigin = std::make_shared<atos_interfaces::srv::GetTestOrigin::Request>();
			originClient->async_send_request(requestOrigin, originCallback);

			// Wait for all services to finish
			std::future<bool> trajLoadedFuture = trajLoaded.get_future();
			std::future<bool> ipLoadedFuture = ipLoaded.get_future();
			std::future<bool> originLoadedFuture = originLoaded.get_future();
			if (auto status = trajLoadedFuture.wait_for(5s); status != std::future_status::ready || !trajLoadedFuture.get()) {
				RCLCPP_ERROR(get_logger(), "Trajectory loading failed for object ID %u", id);
				successful = false;
			}
			if (auto status = ipLoadedFuture.wait_for(250ms); status != std::future_status::ready || !ipLoadedFuture.get()) {
				RCLCPP_ERROR(get_logger(), "IP loading failed for object ID %u", id);
				successful = false;
			}
			if (auto status = originLoadedFuture.wait_for(250ms); status != std::future_status::ready || !originLoadedFuture.get()) {
				RCLCPP_ERROR(get_logger(), "Origin loading failed for object ID %u", id);
				successful = false;
			}
			// Clean up if any of the services failed
			if (!successful) {
				trajectoryClient->prune_pending_requests();
				ipClient->prune_pending_requests();
				originClient->prune_pending_requests();
				return;
			}

		}

		scenarioLoaded.set_value(successful);
	};

	auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();
	auto future = idClient->async_send_request(request, idsCallback);
	// Wait for all objects to load
	std::future scenarioLoadedFuture = scenarioLoaded.get_future();
	if (auto status = scenarioLoadedFuture.wait_for(10s); status == std::future_status::ready) {
		return scenarioLoadedFuture.get();	// Get the status value
	} else if (status == std::future_status::timeout) {
		RCLCPP_ERROR(get_logger(), "Scenario loading timed out");
		idClient->prune_pending_requests();
		return false;
	} else if (status == std::future_status::deferred) {
		RCLCPP_ERROR(get_logger(), "Scenario loading is deferred");
		idClient->prune_pending_requests();
		return false;
	}
	// This should never happen
	return false;
}

void ObjectControl::loadObjectFiles() {
	char path[MAX_FILE_PATH];
	std::vector<std::invalid_argument> errors;

	UtilGetObjectDirectoryPath(path, sizeof (path));
	fs::path objectDir(path);
	if (!fs::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	for (const auto& entry : fs::directory_iterator(objectDir)) {
		if (fs::is_regular_file(entry.status())) {
			const auto inputFile = entry.path();
			ObjectConfig conf(get_logger());
			try {
				conf.parseConfigurationFile(inputFile);
				uint32_t id = conf.getTransmitterID();
				RCLCPP_INFO(get_logger(), "Loaded configuration: %s", conf.toString().c_str());
				// Check preexisting
				auto foundObject = objects.find(id);
				if (foundObject == objects.end()) {
					std::shared_ptr<TestObject> object = std::make_shared<TestObject>(id);
					object->parseConfigurationFile(inputFile);
					objects.emplace(id, object);
				}
				else {
					auto badID = conf.getTransmitterID();
					std::string errMsg = "Duplicate object ID " + std::to_string(badID)
							+ " detected in files " + objects.at(badID)->getTrajectoryFileName()
							+ " and " + conf.getTrajectoryFileName();
					throw std::invalid_argument(errMsg);
				}
			}
			catch (std::invalid_argument& e) {
				RCLCPP_ERROR(get_logger(), e.what());
				errors.push_back(e);
			}			
		}
	}

	// Fix injector ID maps - reverse their direction
	for (const auto& id : getVehicleIDs()) {
		auto injMap = objects.at(id)->getObjectConfig().getInjectionMap();
		for (const auto& sourceID : injMap.sourceIDs) {
			auto conf = objects.at(sourceID)->getObjectConfig();
			conf.addInjectionTarget(id);
			objects.at(sourceID)->setObjectConfig(conf);
		}
	}
	
	if (!errors.empty()) {
		objects.clear();
		std::ostringstream ostr;
		auto append = [&ostr](const std::invalid_argument& e){
			ostr << e.what() << std::endl;
		};
		std::for_each(errors.begin(), errors.end(), append);
		throw std::invalid_argument("Failed to parse object file(s):\n" + ostr.str());
	}
}

uint32_t ObjectControl::getAnchorObjectID() const {
	for (auto& object : objects) {
		if (object.second->isAnchor()) {
			return object.first;
		}
	}
	throw std::invalid_argument("No configured anchor object found");
}

ObjectMonitorType ObjectControl::getLastAnchorData() const {
	auto anchorID = getAnchorObjectID();
	return objects.at(anchorID)->getLastMonitorData();
}

std::map<uint32_t,ObjectStateType> ObjectControl::getObjectStates() const {
	std::map<uint32_t, ObjectStateType> retval;
	for (const auto& elem : objects) {
		retval[elem.first] = elem.second->getState();
	}
	return retval;
}

void ObjectControl::transformScenarioRelativeTo(
		const uint32_t objectID) {
	for (auto& id : getVehicleIDs()) {
		if (id == objectID) {
			// Skip for now TODO also here - maybe?
			continue;
		}
		auto traj = objects.at(id)->getTrajectory();
		auto relTraj = traj.relativeTo(objects.at(objectID)->getTrajectory());

		objects.at(id)->setTrajectory(relTraj);
	}
}

void ObjectControl::clearScenario() {
	objects.clear();
	storedActions.clear();
}


void ObjectControl::beginConnectionAttempt() {
	connStopReqPromise = std::promise<void>();
	connStopReqFuture = connStopReqPromise.get_future();

	RCLCPP_DEBUG(get_logger(), "Initiating connection attempt");
	for (const auto id : getVehicleIDs()) {
		auto t = std::thread(&ObjectControl::connectToObject, this,
							 std::ref(objects.at(id)),
							 std::ref(connStopReqFuture));
		t.detach();
	}
}

void ObjectControl::abortConnectionAttempt() {
	try {
		connStopReqPromise.set_value();
	}
	catch (std::future_error&) {
		// Attempted to stop when none in progress
	}
}

void ObjectControl::disconnectObjects() {
	abortConnectionAttempt();
	try {
		stopHeartbeatSignal.set_value();
	}
	catch (std::future_error&) {
		// Attempted to stop when none in progress
	}
	objectListeners.clear();
	for (const auto id : getVehicleIDs()) {
		objects.at(id)->disconnect();
	}
}

void ObjectControl::disconnectObject(
		const uint32_t id) {
	objects.at(id)->disconnect();
	objectListeners.erase(id);
}

void ObjectControl::uploadAllConfigurations() {
	for (auto& id : getVehicleIDs()) {
		this->uploadObjectConfiguration(id);
	}
}

void ObjectControl::uploadObjectConfiguration(
		const uint32_t id) {
	objects.at(id)->sendSettings();
}

void ObjectControl::startSafetyThread() {
	stopHeartbeatSignal = std::promise<void>();
	if (safetyThread.joinable()) {
		safetyThread.join();
	}
	safetyThread = std::thread(&ObjectControl::heartbeat, this);
}

void ObjectControl::heartbeat() {
	auto stopRequest = stopHeartbeatSignal.get_future();
	clock::time_point nextHeartbeat = clock::now();

	RCLCPP_DEBUG(get_logger(), "Starting heartbeat thread");
	while (stopRequest.wait_until(nextHeartbeat) == std::future_status::timeout) {
		nextHeartbeat += heartbeatPeriod;

		// Check time since MONR for all objects
		for (const auto& id : getVehicleIDs()) {
			if (objects.at(id)->isConnected()) {
				auto diff = objects.at(id)->getTimeSinceLastMonitor();
				if (diff > objects.at(id)->getMaxAllowedMonitorPeriod()) {
					RCLCPP_WARN(get_logger(), "MONR timeout for object %u: %ld ms > %ld ms", id,
							   diff.count(), objects.at(id)->getMaxAllowedMonitorPeriod().count());
					objects.at(id)->disconnect();
					this->state->disconnectedFromObject(*this, id);
				}
			}
		}

		// Send heartbeat
		for (const auto& id : getVehicleIDs()) {
			try {
				if (objects.at(id)->isConnected()) {
					objects.at(id)->sendHeartbeat(this->state->asControlCenterStatus());
				}
			}
			catch (std::exception& e) {
				RCLCPP_WARN(get_logger(), e.what());
				objects.at(id)->disconnect();
				this->state->disconnectedFromObject(*this, id);
			}
		}
	}
	RCLCPP_INFO(get_logger(), "Heartbeat thread exiting");
}

OsiHandler::LocalObjectGroundTruth_t ObjectControl::buildOSILocalGroundTruth(
		const MonitorMessage& monr) const {

	OsiHandler::LocalObjectGroundTruth_t gt;

	gt.id = monr.first;
	gt.pos_m.x = monr.second.position.xCoord_m;
	gt.pos_m.y = monr.second.position.yCoord_m;
	gt.pos_m.z = monr.second.position.zCoord_m;
	gt.vel_m_s.lon = monr.second.speed.isLongitudinalValid ? monr.second.speed.longitudinal_m_s : 0.0;
	gt.vel_m_s.lat = monr.second.speed.isLateralValid ? monr.second.speed.lateral_m_s : 0.0;
	gt.vel_m_s.up = 0.0;
	gt.acc_m_s2.lon = monr.second.acceleration.isLongitudinalValid ? monr.second.acceleration.longitudinal_m_s2 : 0.0;
	gt.acc_m_s2.lat = monr.second.acceleration.isLateralValid ? monr.second.acceleration.lateral_m_s2 : 0.0;
	gt.acc_m_s2.up = 0.0;
	gt.orientation_rad.yaw = monr.second.position.isHeadingValid ? monr.second.position.heading_rad : 0.0;
	gt.orientation_rad.roll = 0.0;
	gt.orientation_rad.pitch = 0.0;

	return gt;
}

// TODO (NOT WORKING ATM!): Replace this with a subscriber that listens for monr messages. 
void ObjectControl::injectObjectData(const MonitorMessage &monr) {
	if (!objects.at(monr.first)->getObjectConfig().getInjectionMap().targetIDs.empty()) {
		std::chrono::system_clock::time_point ts;
		auto secs = std::chrono::seconds(monr.second.timestamp.tv_sec);
		auto usecs = std::chrono::microseconds(monr.second.timestamp.tv_usec);
		ts += secs + usecs;
		auto osiGtData = buildOSILocalGroundTruth(monr);
		for (const auto& targetID : objects.at(monr.first)->getObjectConfig().getInjectionMap().targetIDs) {
			if (objects.at(targetID)->isOsiCompatible()) {
				objects.at(targetID)->sendOsiData(osiGtData, objects.at(targetID)->getProjString(), ts);
			}
		}
	}
}


void ObjectControl::startListeners() {
	RCLCPP_DEBUG(get_logger(), "Starting listeners");
	objectListeners.clear();
	for (const auto& id : getVehicleIDs()) {
		objectListeners.try_emplace(id, this, objects.at(id), get_logger());
	}
}

void ObjectControl::publishObjectIds() {
	atos_interfaces::msg::ObjectIdArray msg;
	msg.ids = getVehicleIDs();
	connectedObjectIdsPub.publish(msg);
}

void ObjectControl::notifyObjectsConnected() {
	objectsConnectedPub.publish(ROSChannels::ObjectsConnected::message_type());
}

void ObjectControl::connectToObject(
		std::shared_ptr<TestObject> obj,
		std::shared_future<void> &connStopReq) {
	const int maxConnHeabs = this->get_parameter("max_missing_heartbeats").as_int();
	constexpr int maxConnMonrs = 100;
	try {
		if (!obj->isConnected()) {
			try {
				obj->establishConnection(connStopReq);
				obj->sendSettings();
			}
			catch (std::runtime_error& e) {
				RCLCPP_ERROR(get_logger(), "Connection attempt for object %u failed: %s",
						   obj->getTransmitterID(), e.what());
				obj->disconnect();
				return;
				// TODO connection failed event?
			}
			try {
				int initializingMonrs = maxConnMonrs;
				int connectionHeartbeats = maxConnHeabs;

				while (true) {
					ObjectStateType objState = OBJECT_STATE_UNKNOWN;
					auto nextSendTime = std::chrono::system_clock::now();
					try {
						obj->sendHeartbeat(this->state->asControlCenterStatus());
						nextSendTime += heartbeatPeriod;
						objState = obj->getState(true, heartbeatPeriod);
					} catch (std::runtime_error& e) {
						if (connectionHeartbeats-- > 0) {
							std::this_thread::sleep_until(nextSendTime);
							continue;
						}
						else {
							throw std::runtime_error("No monitor reply after " + std::to_string(maxConnHeabs) + " heartbeats. Details:\n" + e.what());
						}
					}
					switch (objState) {
					case OBJECT_STATE_ARMED:
					case OBJECT_STATE_REMOTE_CONTROL:
						RCLCPP_INFO(get_logger(), "Connected to armed object ID %u", obj->getTransmitterID());
						this->state->connectedToArmedObject(*this, obj->getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
					case OBJECT_STATE_POSTRUN:
					case OBJECT_STATE_RUNNING:
						RCLCPP_INFO(get_logger(), "Connected to running object ID %u", obj->getTransmitterID());
						this->state->connectedToLiveObject(*this, obj->getTransmitterID());
						break;
					case OBJECT_STATE_INIT:
						if (initializingMonrs-- > 0) {
							continue;
						}
						else {
							RCLCPP_INFO(get_logger(), "Connected object %u in initializing state after connection", obj->getTransmitterID());
							this->state->connectedToLiveObject(*this, obj->getTransmitterID());
						}
						break;
					case OBJECT_STATE_DISARMED:
						RCLCPP_INFO(get_logger(), "Connected to disarmed object ID %u", obj->getTransmitterID());
						this->state->connectedToObject(*this, obj->getTransmitterID());
						break;
					default:
						RCLCPP_INFO(get_logger(), "Connected to object %u in unknown state", obj->getTransmitterID());
						this->state->connectedToLiveObject(*this, obj->getTransmitterID());
					}
					break;
				}
			}
			catch (std::runtime_error &e) {
				RCLCPP_ERROR(get_logger(), "Connection startup procedure failed for object %u: %s",
						   obj->getTransmitterID(), e.what());
				obj->disconnect();
				// TODO: connection failed event?
			}
		}
	}
	catch (std::invalid_argument &e) {
		RCLCPP_ERROR(get_logger(), "Bad connection attempt for object %u: %s",
				   obj->getTransmitterID(), e.what());
		obj->disconnect();
		// TODO: connection failed event?
	}
};

void ObjectControl::remoteControlObjects(bool on) {
	for (auto& id : getVehicleIDs()) {
		objects.at(id)->sendRemoteControl(on);
	}
}

void ObjectControl::armObjects() {
	for (auto& id : getVehicleIDs()) {
		objects.at(id)->sendArm();
	}
}

void ObjectControl::disarmObjects() {
	for (auto& id : getVehicleIDs()) {
		try {
			objects.at(id)->sendDisarm();
		}
		catch (std::invalid_argument& e) {
			RCLCPP_ERROR(get_logger(), "Unable to disarm object %u: %s", id, e.what());
		}
		catch (std::runtime_error& e) {
			RCLCPP_ERROR(get_logger(), "Unable to disarm object %u: %s", id, e.what());
			objects.at(id)->disconnect();
		}
	}
}

/**
 * @brief Resets the scenario by offering return trajectories for all objects
 * 
 * @param isResetting state-flag to indicate if the reset procedure is currently running
*/
void ObjectControl::resetTestObjects() {
	// Check if we are already resetting
	if (this->isResetting) {
		RCLCPP_INFO(get_logger(), "Test already resetting");
		return;
	}
	this->isResetting = true;
	RCLCPP_INFO(get_logger(), "Resetting test by offering return trajectories for all objects");
	// Request a new return trajectory for each object
	for (auto& id : getVehicleIDs()) {
		this->setObjectTrajectory(id);
	}
}

/**
 * @brief Reloads the scenario trajectories for all objects
 * 
 * @param isResetting state-flag to indicate if the reset procedure is currently running
 */
void ObjectControl::reloadScenarioTrajectories() {
	// Check if we already have loaded the scenario trajectories
	if (!this->isResetting) {
		RCLCPP_INFO(get_logger(), "Scenario trajectories already loaded");
		return;
	}
	this->isResetting = false;
	RCLCPP_INFO(get_logger(), "Reloading scenario trajectories");
	// Request the scenario trajectory for each object
	for (auto& id : getVehicleIDs()) {
		this->setObjectTrajectory(id);
	}
}

/**
 * @brief Republishes the trajectory paths for a given object
 * 
 * @param id the object id
*/
void ObjectControl::republishTrajectoryPaths(uint32_t id){
	// Update the GUI with the new trajectory in local coordinates
	ATOS::Trajectory traj = objects.at(id)->getTrajectory();
	this->pathPublishers.emplace(id, ROSChannels::Path::Pub(*this, id));
	this->pathPublishers.at(id).publish(traj.toPath());

	// Update the GUI with the new trajectory in global coordinates
	GeographicPositionType origin = objects.at(id)->getOrigin();
	std::array<double,3> llh_0 = {origin.latitude_deg, origin.longitude_deg, origin.altitude_m};
	this->gnssPathPublishers.emplace(id, ROSChannels::GNSSPath::Pub(*this, id));
	this->gnssPathPublishers.at(id).publish(traj.toGeoJSON(llh_0));
}

/**
 * @brief Callback function for the trajectory service call. Sets the new trajectory for the object in ATOS and sends it to the object
 * 
 * @param future the future object containing the response from the trajectory service call
*/
void ObjectControl::trajectoryCallback(const rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedFuture future) {
	this->trajResponse = future.get();
	auto id = returnTrajResponse->id;
	// Check if the return trajectory service call was successful
	if (!trajResponse->success) {
		RCLCPP_ERROR(get_logger(), "Get trajectory service call failed for object %u", id);
		return;
	}

	// Fill the trajectory class with the received trajectory and send to the object
	ATOS::Trajectory traj(get_logger());
	traj.initializeFromCartesianTrajectory(trajResponse->trajectory);
	objects.at(id)->setTrajectory(traj);
	objects.at(id)->sendTrajectory();
	// Update the GUI with the new trajectory
	this->republishTrajectoryPaths(id);
	RCLCPP_INFO(get_logger(), "Loaded trajectory for object %u with %lu points", id, traj.size());
};

/**
 * @brief Callback function for the return trajectory service call. Sets the new trajectory for the object in ATOS and sends it to the object
 * 
 * @param future the future object containing the response from the return trajectory service call
*/
void ObjectControl::returnTrajectoryCallback(const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future) {
	this->returnTrajResponse = future.get();
	auto id = returnTrajResponse->id;
	// Check if the return trajectory service call was successful
	if (!returnTrajResponse->success) {
		RCLCPP_ERROR(get_logger(), "Get return trajectory service call failed for object %u", id);
		return;
	}

	// Fill the trajectory class with the received trajectory and send to the object
	ATOS::Trajectory traj(get_logger());
	traj.initializeFromCartesianTrajectory(returnTrajResponse->trajectory);
	objects.at(id)->setTrajectory(traj);
	objects.at(id)->sendTrajectory();
	// Update the GUI with the new trajectory
	this->republishTrajectoryPaths(id);
	RCLCPP_INFO(get_logger(), "Loaded return trajectory for object %u with %lu points", id, traj.size());
};

/**
 * @brief Sets the trajectory for a given object by requesting it from the esminiAdapter or BackToStart module depending on the current state (resetting)
 * 
 * @param id the object id
*/
void ObjectControl::setObjectTrajectory(uint32_t id){
	// Check if object is in resetting "state". If so, offer return trajectory instead of normal trajectory
	if (this->isResetting) {
		using namespace std::chrono_literals;
		if (!this->returnTrajectoryClient->wait_for_service(1s)) // TODO: This is blocking any other callbacks for 1s if the service is not available. Consider new thread.
		{
			RCLCPP_ERROR(get_logger(), "The return trajectory service is not available. Try again later.");
			return;
		}
		// Get the current trajectory and create a request for the return trajectory service
		auto returnTrajectoryRequest = std::make_shared<atos_interfaces::srv::GetObjectReturnTrajectory::Request>();
		returnTrajectoryRequest->id = id;
		returnTrajectoryRequest->trajectory = objects.at(id)->getTrajectory().toCartesianTrajectory();
		auto pos = objects.at(id)->getLastMonitorData().position;
		returnTrajectoryRequest->position.x = pos.xCoord_m;
		returnTrajectoryRequest->position.y = pos.yCoord_m;
		returnTrajectoryRequest->position.z = pos.zCoord_m;
		// Send the request to the return trajectory service asynchronously and bind the callback function for when the response is received
		returnTrajectoryClient->async_send_request(returnTrajectoryRequest, std::bind(&ObjectControl::returnTrajectoryCallback, this, _1));
	}
	else {
		// Request the scenario trajectory from the esminiAdapter
		auto trajRequest = std::make_shared<atos_interfaces::srv::GetObjectTrajectory::Request>();
		trajRequest->id = id;
		trajectoryClient->async_send_request(trajRequest, std::bind(&ObjectControl::trajectoryCallback, this, _1));
	}
}

void ObjectControl::startObject(
	uint32_t id,
	std::chrono::system_clock::time_point startTime) {
	try {
			objects.at(id)->sendStart(startTime);
	}
	catch (std::out_of_range& e) {
		std::stringstream ss;
		for (auto& id : getVehicleIDs()) {
			ss << " " << id << ",";
		}
		auto str = ss.str();
		str.pop_back();
		RCLCPP_WARN(get_logger(), "Attempted to start nonexistent object %u - configured objects are%s", id, str.c_str());
	}
}

void ObjectControl::onObjectStateChangeMessage(const ObjectStateChange::message_type::SharedPtr stateChangeMsg) {
	ObjectStateType prevState = static_cast<ObjectStateType>(stateChangeMsg->prev_state.state);
	ObjectStateType state = static_cast<ObjectStateType>(stateChangeMsg->state.state);
	uint32_t id = stateChangeMsg->id;
	switch (state) {
	case OBJECT_STATE_DISARMED:
		if (prevState == OBJECT_STATE_ABORTING) {
			this->state->objectAbortDisarmed(*this, id);
		}
		else {
			this->state->objectDisarmed(*this, id);
		}
		break;
	case OBJECT_STATE_ARMED:
		this->state->objectArmed(*this, id);
		break;
	case OBJECT_STATE_ABORTING:
		this->state->objectAborting(*this, id);
		break;
	case OBJECT_STATE_INIT:
	case OBJECT_STATE_RUNNING:
	case OBJECT_STATE_POSTRUN:
	case OBJECT_STATE_REMOTE_CONTROL:
		break;

	default:
		RCLCPP_WARN(get_logger(), "Received unknown object state change message for object %u: %d", id, state);
	}
}

void ObjectControl::allClearObjects() {
	// Send allClear to all connected objects
	for (auto& id : getVehicleIDs()) {
		if (objects.at(id)->isConnected()) {
			objects.at(id)->sendAllClear();
		}
	}
}

bool ObjectControl::areAllObjects(std::function<bool(const std::shared_ptr<TestObject>)> predicate) const {
	return std::all_of(objects.cbegin(), objects.cend(), [predicate](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return predicate(obj.second);
	});
}

bool ObjectControl::isAnyObject(std::function<bool(const std::shared_ptr<TestObject>)> predicate) const {
	return std::any_of(objects.cbegin(), objects.cend(), [predicate](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return predicate(obj.second);
	});
}

bool ObjectControl::isAnyObjectIn(
		const ObjectStateType state) {
	return std::any_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return obj.second->getState() == state;
	});
}

bool ObjectControl::areAllObjectsIn(
		const ObjectStateType state) {
	return std::all_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return obj.second->getState() == state;
	});
}

bool ObjectControl::isAnyObjectIn(
		const std::set<ObjectStateType>& states) {
	return std::any_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return states.find(obj.second->getState()) != states.end();
	});
}

bool ObjectControl::areAllObjectsIn(
		const std::set<ObjectStateType>& states) {
	return std::all_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& obj) {
		return states.find(obj.second->getState()) != states.end();
	});
}

void ObjectControl::startControlSignalSubscriber(){
	controlSignalSub = std::make_shared<ControlSignal::Sub>(*this, std::bind(&ObjectControl::onControlSignalMessage, this, _1));
}
void ObjectControl::stopControlSignalSubscriber(){
	this->controlSignalSub.reset();
}

void ObjectControl::sendAbortNotification(){
	this->scnAbortPub.publish(ROSChannels::Abort::message_type());
}

/**
 * @brief Publishes scenario info to the journal
 * 
 */
void ObjectControl::publishScenarioInfoToJournal() {
	JournalRecordData(JOURNAL_RECORD_STRING, "--- Scenario Info ---");
	std::stringstream ss;
	int index = 1;
	for (const auto& object : objects) {
		auto testObject = object.second;
		auto traj = testObject->getTrajectory();

		// Convert binary IP to string for logging
		auto ip = testObject->getAsObjectData().ClientIP;
		char ip_str[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &ip, ip_str, INET_ADDRSTRLEN);

		ss << "\n> Object " << index << ": \n"
			 << "\t - ID: " << object.first << "\n" 
			 << "\t - IP: " << ip_str << "\n"
			 << "\t - Origin: (" << testObject->getOrigin().latitude_deg << ", " << testObject->getOrigin().longitude_deg << ", " << testObject->getOrigin().altitude_m << ")\n"
			 << "\t - Trajectory size: " << testObject->getTrajectory().size() << "\n"
			 << "\t - Trajectory: \n";
		for (const auto& point : traj.points) {
			ss << "\t\t" << point.toString() << "\n";
		}

		++index;

	}
	JournalRecordData(JOURNAL_RECORD_STRING, ss.str().c_str());

	// print params.yaml in journal
	auto path = getenv("HOME") + std::string("/.astazero/ATOS/conf/params.yaml");
	std::ifstream ifs(path);
	std::stringstream params;
	params << "Parameters from params.yaml: \n" << ifs.rdbuf();
	JournalRecordData(JOURNAL_RECORD_STRING, params.str().c_str());
	JournalRecordData(JOURNAL_RECORD_STRING, "--- End of Scenario Info ---");
}
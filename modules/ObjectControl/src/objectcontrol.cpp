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

#include "state.hpp"
#include "util.h"
#include "journal.h"
#include "datadictionary.h"
#include "atos_interfaces/srv/get_start_on_trigger.hpp"

#include "objectcontrol.hpp"

using std::placeholders::_1;
using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace ATOS;

ObjectControl::ObjectControl()
	: Module(ObjectControl::moduleName),
	scnInitSub(*this, std::bind(&ObjectControl::onInitMessage, this, _1)),
	scnStartSub(*this, std::bind(&ObjectControl::onStartMessage, this, _1)),
	scnArmSub(*this, std::bind(&ObjectControl::onArmMessage, this, _1)),
	scnStopSub(*this, std::bind(&ObjectControl::onStopMessage, this, _1)),
	scnAbortSub(*this, std::bind(&ObjectControl::onAbortMessage, this, _1)),
	scnAllClearSub(*this, std::bind(&ObjectControl::onAllClearMessage, this, _1)),
	scnConnectSub(*this, std::bind(&ObjectControl::onConnectMessage, this, _1)),
	scnDisconnectSub(*this, std::bind(&ObjectControl::onDisconnectMessage, this, _1)),
	scnActionSub(*this, std::bind(&ObjectControl::onEXACMessage, this, _1)),
	scnActionConfigSub(*this, std::bind(&ObjectControl::onACCMMessage, this, _1)),
	getStatusSub(*this, std::bind(&ObjectControl::onGetStatusMessage, this, _1)),
	scnRemoteControlEnableSub(*this, std::bind(&ObjectControl::onRemoteControlEnableMessage, this, _1)),
	scnRemoteControlDisableSub(*this, std::bind(&ObjectControl::onRemoteControlDisableMessage, this, _1)),
	failurePub(*this),
	scnAbortPub(*this),
	objectsConnectedPub(*this),
	connectedObjectIdsPub(*this)
{
	declare_parameter("load_ros_cartesian_trajectory",false);
	int queueSize=0;
	objectsConnectedTimer = create_wall_timer(1000ms, std::bind(&ObjectControl::publishObjectIds, this));
    idClient = create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds);
	trajectoryClient = create_client<atos_interfaces::srv::GetObjectTrajectory>(ServiceNames::getObjectTrajectory);
	ipClient = create_client<atos_interfaces::srv::GetObjectIp>(ServiceNames::getObjectIp);
	triggerClient = create_client<atos_interfaces::srv::GetObjectTriggerStart>(ServiceNames::getObjectTriggerStart);
	if (this->initialize() == -1) {
		throw std::runtime_error(std::string("Failed to initialize ") + get_name());
	}
};

ObjectControl::~ObjectControl() {
	delete state;
}

int ObjectControl::initialize() {
	int retval = 0;

	// Initialize log
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d",get_name(), getpid());

	// Create test journal
	if (JournalInit(get_name()) == -1) {
		retval = -1;
		RCLCPP_ERROR(get_logger(), "Unable to create test journal");
	}

	if (requestDataDictInitialization()) {
		// Map state and object data into memory
		if (DataDictionaryInitObjectData() != READ_OK) {
			DataDictionaryFreeObjectData();
			retval = -1;
			RCLCPP_ERROR(get_logger(),
						"Found no previously initialized shared memory for object data");
		}
		if (DataDictionaryInitStateData() != READ_OK) {
			DataDictionaryFreeStateData();
			retval = -1;
			RCLCPP_ERROR(get_logger(),
						"Found no previously initialized shared memory for state data");
		}
		else {
			// Set state
			this->state = static_cast<ObjectControlState*>(new AbstractKinematics::Idle);
			DataDictionarySetOBCState(this->state->asNumber());
		}
	}
	else {
		retval = -1;
		RCLCPP_ERROR(get_logger(), "Unable to initialize data dictionary");
	}
	return retval;
}

void ObjectControl::handleActionConfigurationCommand(
		const TestScenarioCommandAction& action) {
	this->state->settingModificationRequested(*this);
	if (action.command == ACTION_PARAMETER_VS_SEND_START) {
		RCLCPP_INFO(get_logger(), "Configuring delayed start for object %u", action.objectID);
		objects.at(action.objectID)->setTriggerStart(true);
		this->storedActions[action.actionID] = std::bind(&TestObject::sendStart,
														 objects.at(action.objectID));
	}
}

void ObjectControl::handleExecuteActionCommand(
		const uint16_t &actionID,
		const std::chrono::system_clock::time_point &when) {
	this->state->actionExecutionRequested(*this);
	auto delayedExecutor = [&](){
		using namespace std::chrono;
		RCLCPP_DEBUG(get_logger(), "Executing action %u in %d ms", actionID,
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
	auto f_try = [&]() { this->state->initializeRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Init::topicName, get_logger());
}

void ObjectControl::onConnectMessage(const Connect::message_type::SharedPtr){	
	COMMAND cmd = COMM_CONNECT;
	auto f_try = [&]() { this->state->connectRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Connect::topicName, get_logger());
}

void ObjectControl::onArmMessage(const Arm::message_type::SharedPtr){	
	COMMAND cmd = COMM_ARM;
	auto f_try = [&]() { this->state->armRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Arm::topicName, get_logger());
}

void ObjectControl::onStartMessage(const Start::message_type::SharedPtr){	
	COMMAND cmd = COMM_STRT;
	auto f_try = [&]() { this->state->startRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Start::topicName, get_logger());
}

void ObjectControl::onDisconnectMessage(const Disconnect::message_type::SharedPtr){	
	COMMAND cmd = COMM_DISCONNECT;
	auto f_try = [&]() { this->state->disconnectRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, Disconnect::topicName, get_logger());
}

void ObjectControl::onStopMessage(const Stop::message_type::SharedPtr){
	COMMAND cmd = COMM_STOP;
	auto f_try = [&]() { this->state->stopRequest(*this); };
	auto f_catch = [&]() {
			failurePub.publish(msgCtr1<Failure::message_type>(cmd));
			scnAbortPub.publish(Abort::message_type());
	};
	this->tryHandleMessage(f_try,f_catch, Stop::topicName, get_logger());	
}

void ObjectControl::onAbortMessage(const Abort::message_type::SharedPtr){	
	// Any exceptions here should crash the program
	this->state->abortRequest(*this);
}

void ObjectControl::onAllClearMessage(const AllClear::message_type::SharedPtr){	
	COMMAND cmd = COMM_ABORT_DONE;
	auto f_try = [&]() { this->state->allClearRequest(*this); };
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, AllClear::topicName, get_logger());
}

void ObjectControl::onACCMMessage(const ActionConfiguration::message_type::SharedPtr accm){
	COMMAND cmd = COMM_ACCM;
	auto f_try = [&]() {
		if (accm->action_type == ACTION_TEST_SCENARIO_COMMAND) {
			ObjectControl::TestScenarioCommandAction cmdAction;
			cmdAction.command = static_cast<ActionTypeParameter_t>(accm->action_type_parameter1);
			cmdAction.actionID = accm->action_id;
			cmdAction.objectID = getVehicleIDByIP(accm->ip);
			handleActionConfigurationCommand(cmdAction);
		}
	};
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, ActionConfiguration::topicName, get_logger());
}

void ObjectControl::onEXACMessage(const ExecuteAction::message_type::SharedPtr exac){
	COMMAND cmd = COMM_EXAC;
	auto f_try = [&]() {
		using namespace std::chrono;
		quartermilliseconds qmsow(exac->executiontime_qmsow);
		auto now = to_timeval(system_clock::now().time_since_epoch());
		auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
		handleExecuteActionCommand(exac->action_id, startOfWeek+qmsow);	
	};
	auto f_catch = [&]() { failurePub.publish(msgCtr1<Failure::message_type>(cmd)); };
	this->tryHandleMessage(f_try,f_catch, ExecuteAction::topicName, get_logger());
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

void ObjectControl::loadScenario() {
	this->clearScenario();

	auto idsCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedFuture future) {
		auto idResponse = future.get();
		RCLCPP_ERROR(get_logger(), "Got %d object ids", idResponse->ids.size());

		for (auto id : idResponse->ids) {
			auto trajletSub = std::make_shared<Path::Sub>(*this, id, std::bind(&ObjectControl::onPathMessage, this, _1, id));
			auto monrPub = std::make_shared<Monitor::Pub>(*this, id);
			objects[id] = std::make_shared<TestObject>(this->get_logger(), trajletSub, monrPub);
			objects.at(id)->setTransmitterID(id);

			auto trajectoryCallback = [id, this](const rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedFuture future) {
				auto trajResponse = future.get();
				//trajResponse->
				RCLCPP_ERROR(get_logger(), "Got trajectory for object %d", id);
				ATOS::Trajectory traj;
				traj.initializeFromCartesianTrajectory(trajResponse->trajectory);
				objects.at(id)->setTrajectory(traj);
			};
			auto trajRequest = std::make_shared<atos_interfaces::srv::GetObjectTrajectory::Request>();
			trajRequest->id = id;
			trajectoryClient->async_send_request(trajRequest, trajectoryCallback);

			auto ipCallback = [id, this](const rclcpp::Client<atos_interfaces::srv::GetObjectIp>::SharedFuture future) {
				auto ipResponse = future.get();
				// convert from string to in_addr
				in_addr_t ip;
				inet_pton(AF_INET, ipResponse->ip.c_str(), &ip);
				objects.at(id)->setObjectIP(ip);
			};

			auto ipRequest = std::make_shared<atos_interfaces::srv::GetObjectIp::Request>();
			ipRequest->id = id;
			ipClient->async_send_request(ipRequest, ipCallback);
		}
	};

	// TODO query scenario participants from scenario manager
	auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();
	auto promise = idClient->async_send_request(request, idsCallback);
	return;
	// TODO load trajectories for each participant
	// TODO configure delayed start

	this->loadObjectFiles();
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, std::shared_ptr<TestObject>> &o) {
		auto data = o.second->getAsObjectData();
		DataDictionarySetObjectData(&data);
	});
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
			ObjectConfig conf;
			try {
				conf.parseConfigurationFile(inputFile,false);
				uint32_t id = conf.getTransmitterID();
				RCLCPP_INFO(get_logger(), "Loaded configuration: %s", conf.toString().c_str());
				// Check preexisting
				auto foundObject = objects.find(id);
				if (foundObject == objects.end()) {
					// Create sub and pub as unique ptrs, when TestObject is destroyed, these get destroyed too.
					
					auto trajletSub = std::make_shared<Path::Sub>(*this, id, std::bind(&ObjectControl::onPathMessage, this, _1, id));
					auto monrPub = std::make_shared<Monitor::Pub>(*this, id);
					std::shared_ptr<TestObject> object = std::make_shared<TestObject>(get_logger(),trajletSub,monrPub);
					auto getTrajFromRos = get_parameter("use_ros_trajectory").as_bool();
					if (getTrajFromRos){ // TODO: This is a temp solution.. using ros/openscenario we can probably skip having to parse .conf and .opro files
						//auto response = std::shared_ptr<GetTrajectory::Response>;
						//nShotServiceRequest(3,1s,"get_trajectory",response);
						//object->getTrajectory().initializeFromCartesianTrajectory(response->trajectory);

					}
					object->parseConfigurationFile(inputFile, getTrajFromRos);
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
	catch (std::future_error) {
		// Attempted to stop when none in progress
	}
}

void ObjectControl::disconnectObjects() {
	abortConnectionAttempt();
	try {
		stopHeartbeatSignal.set_value();
	}
	catch (std::future_error) {
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
					RCLCPP_WARN(get_logger(), "MONR timeout for object %u: %d ms > %d ms", id,
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
			catch (std::invalid_argument& e) {
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
	constexpr int maxConnHeabs = 10;
	constexpr int maxConnMonrs = 10;
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
	}
	this->state->allObjectsDisarmed(*this); // TODO add a check on object states as well
}

void ObjectControl::startObjects() {
	for (auto& id : getVehicleIDs()) {
		if (!objects.at(id)->isStartingOnTrigger()) {
			objects.at(id)->sendStart();
		}
	}
}

void ObjectControl::allClearObjects() {
	for (auto& id : getVehicleIDs()) {
		objects.at(id)->sendAllClear();
	}
	this->state->allObjectsAbortDisarmed(*this); // TODO wait for all objects really are disarmed
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

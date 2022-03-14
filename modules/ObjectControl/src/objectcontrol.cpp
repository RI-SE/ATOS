#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>
#include <dirent.h>

#include "state.hpp"
#include "logging.h"
#include "util.h"
#include "journal.h"
#include "datadictionary.h"

#include "objectcontrol.hpp"

using std_msgs::msg::Empty;
using std_msgs::msg::String;
using std_msgs::msg::UInt8;

ObjectControl::ObjectControl(LOG_LEVEL logLevel) : Module(ObjectControl::moduleName){
	this->initialize(logLevel);
	int queueSize=0;
	
	// ** Subscriptions
	this->initSub = this->create_subscription<Empty>(topicNames[COMM_INIT], queueSize, std::bind(&ObjectControl::onInitMessage, this, _1));
	this->connectSub = this->create_subscription<Empty>(topicNames[COMM_CONNECT], queueSize, std::bind(&ObjectControl::onConnectMessage, this, _1));
	this->armSub = this->create_subscription<Empty>(topicNames[COMM_ARM], queueSize, std::bind(&ObjectControl::onArmMessage, this, _1));
	this->startSub = this->create_subscription<Empty>(topicNames[COMM_STRT], queueSize, std::bind(&ObjectControl::onStartMessage, this, _1));
	this->disconnectSub = this->create_subscription<Empty>(topicNames[COMM_DISCONNECT], queueSize, std::bind(&ObjectControl::onDisconnectMessage, this, _1));
	this->stopSub = this->create_subscription<Empty>(topicNames[COMM_STOP], queueSize, std::bind(&ObjectControl::onStopMessage, this, _1));
	this->abortSub = this->create_subscription<Empty>(topicNames[COMM_ABORT], queueSize, std::bind(&ObjectControl::onAbortMessage, this, _1));
	this->allClearSub = this->create_subscription<Empty>(topicNames[COMM_ABORT_DONE], queueSize, std::bind(&ObjectControl::onAllClearMessage, this, _1));
	this->accmSub = this->create_subscription<Accm>(topicNames[COMM_ACCM], queueSize, std::bind(&ObjectControl::onACCMMessage, this, _1));
	this->exacSub = this->create_subscription<Exac>(topicNames[COMM_EXAC], queueSize, std::bind(&ObjectControl::onEXACMessage, this, _1));
	this->getStatusSub = this->create_subscription<Empty>(topicNames[COMM_GETSTATUS], queueSize, std::bind(&ObjectControl::onGetStatusMessage, this, _1));
	this->remoteControlEnableSub = this->create_subscription<Empty>(topicNames[COMM_REMOTECTRL_ENABLE], queueSize, std::bind(&ObjectControl::onRemoteControlEnableMessage, this, _1));
	this->remoteControlDisableSub = this->create_subscription<Empty>(topicNames[COMM_REMOTECTRL_DISABLE], queueSize, std::bind(&ObjectControl::onRemoteControlDisableMessage, this, _1));
	this->controlSignalPercentageSub = this->create_subscription<ControlSignalPercentage>(topicNames[COMM_CONTROL_SIGNAL_PERCENTAGE], queueSize, std::bind(&ObjectControl::onControlSignalPercentageMessage, this, _1));

	// ** Publishers
	this->failurePub = this->create_publisher<UInt8>(topicNames[COMM_FAILURE],queueSize);
	this->getStatusResponsePub = this->create_publisher<String>(topicNames[COMM_GETSTATUS_OK],queueSize);	
};

ObjectControl::~ObjectControl() {
	delete state;
}

int ObjectControl::initialize(const LOG_LEVEL logLevel) {
	int retval = 0;

	// Initialize log
	LogInit(get_name(), logLevel);
	LogMessage(LOG_LEVEL_INFO, "%s task running with PID: %d",get_name(), getpid());

	// Create test journal
	if (JournalInit(get_name()) == -1) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to create test journal");
	}

	// Initialize state and object data
	if (DataDictionaryInitStateData() != READ_OK) {
		DataDictionaryFreeStateData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for state data");
	}
	if (DataDictionaryInitObjectData() != READ_OK) {
		DataDictionaryFreeObjectData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for object data");
	}

	// Set state
	this->state = static_cast<ObjectControlState*>(new AbstractKinematics::Idle);
	DataDictionarySetOBCState(this->state->asNumber());

	return retval;
}

void ObjectControl::handleActionConfigurationCommand(
		const TestScenarioCommandAction& action) {
	this->state->settingModificationRequested(*this);
	if (action.command == ACTION_PARAMETER_VS_SEND_START) {
		LogMessage(LOG_LEVEL_INFO, "Configuring delayed start for object %u", action.objectID);
		objects[action.objectID].setTriggerStart(true);
		this->storedActions[action.actionID] = std::bind(&TestObject::sendStart,
														 &objects[action.objectID]);
	}
}

void ObjectControl::handleExecuteActionCommand(
		const uint16_t &actionID,
		const std::chrono::system_clock::time_point &when) {
	this->state->actionExecutionRequested(*this);
	auto delayedExecutor = [&](){
		using namespace std::chrono;
		LogMessage(LOG_LEVEL_DEBUG, "Executing action %u in %d ms", actionID,
				   duration_cast<milliseconds>(when - system_clock::now()).count());
		std::this_thread::sleep_until(when);
		LogMessage(LOG_LEVEL_INFO, "Executing action %u", actionID);
		this->storedActions[actionID]();
	};
	auto thd = std::thread(delayedExecutor);
	thd.detach();
}

void ObjectControl::sendRCMMToObject(
		RemoteControlManoeuvreMessageType& rcmm,
		uint32_t id){
			objects.at(id).sendControlSignal(rcmm);
		}

void ObjectControl::onInitMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_INIT;
	auto f_try = [&]() { this->state->initializeRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onConnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_CONNECT;
	auto f_try = [&]() { this->state->connectRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onArmMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ARM;
	auto f_try = [&]() { this->state->armRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onStartMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_STRT;
	auto f_try = [&]() { this->state->startRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onDisconnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_DISCONNECT;
	auto f_try = [&]() { this->state->disconnectRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onStopMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_STOP;
	auto f_try = [&]() { this->state->stopRequest(*this); };
	auto f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
			abortPub->publish(Empty());
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void ObjectControl::onAbortMessage(const Empty::SharedPtr){	
	// Any exceptions here should crash the program
	this->state->abortRequest(*this);
}

void ObjectControl::onAllClearMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ABORT_DONE;
	auto f_try = [&]() { this->state->allClearRequest(*this); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onACCMMessage(const Accm::SharedPtr accm){
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
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onEXACMessage(const Exac::SharedPtr exac){
	COMMAND cmd = COMM_EXAC;
	auto f_try = [&]() {
		using namespace std::chrono;
		quartermilliseconds qmsow(exac->executiontime_qmsow);
		auto now = to_timeval(system_clock::now().time_since_epoch());
		auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
		handleExecuteActionCommand(exac->action_id, startOfWeek+qmsow);	
	};
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void ObjectControl::onRemoteControlEnableMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_REMOTECTRL_ENABLE;
	auto f_try = [&]() { this->state->enableRemoteControlRequest(*this); };
	auto f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void ObjectControl::onRemoteControlDisableMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_REMOTECTRL_DISABLE;
	auto f_try = [&]() { this->state->disableRemoteControlRequest(*this); };
	auto f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void ObjectControl::onControlSignalPercentageMessage(const ControlSignalPercentage::SharedPtr csp){
	COMMAND cmd = COMM_CONTROL_SIGNAL_PERCENTAGE;
	auto f_try = [&]() { this->state->sendControlSignal(*this,csp); };
	auto f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void ObjectControl::loadScenario() {
	this->loadObjectFiles();
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, TestObject> &o) {
		auto data = o.second.getAsObjectData();
		DataDictionarySetObjectData(&data);
	});
}

void ObjectControl::loadObjectFiles() {
	this->clearScenario();
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
			TestObject object;
			try {
				object.parseConfigurationFile(entry.path());

				LogMessage(LOG_LEVEL_INFO, "Loaded configuration: %s", object.toString().c_str());
				// Check preexisting
				auto foundObject = objects.find(object.getTransmitterID());
				if (foundObject == objects.end()) {
					objects.emplace(object.getTransmitterID(), std::move(object));
				}
				else {
					auto badID = object.getTransmitterID();
					std::string errMsg = "Duplicate object ID " + std::to_string(badID)
							+ " detected in files " + objects[badID].getTrajectoryFileName()
							+ " and " + object.getTrajectoryFileName();
					throw std::invalid_argument(errMsg);
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, e.what());
				errors.push_back(e);
			}
		}
	}

	// Fix injector ID maps - reverse their direction
	for (const auto& id : getVehicleIDs()) {
		auto injMap = objects[id].getObjectConfig().getInjectionMap();
		for (const auto& sourceID : injMap.sourceIDs) {
			auto conf = objects[sourceID].getObjectConfig();
			conf.addInjectionTarget(id);
			objects[sourceID].setObjectConfig(conf);
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
		if (object.second.isAnchor()) {
			return object.first;
		}
	}
	throw std::invalid_argument("No configured anchor object found");
}

ObjectMonitorType ObjectControl::getLastAnchorData() const {
	auto anchorID = getAnchorObjectID();
	return objects.at(anchorID).getLastMonitorData();
}

std::map<uint32_t,ObjectStateType> ObjectControl::getObjectStates() const {
	std::map<uint32_t, ObjectStateType> retval;
	for (const auto& elem : objects) {
		retval[elem.first] = elem.second.getState();
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
		auto traj = objects[id].getTrajectory();
		auto relTraj = traj.relativeTo(objects[objectID].getTrajectory());

		objects[id].setTrajectory(relTraj);
	}
}

void ObjectControl::clearScenario() {
	objects.clear();
	storedActions.clear();
}


void ObjectControl::beginConnectionAttempt() {
	connStopReqPromise = std::promise<void>();
	connStopReqFuture = connStopReqPromise.get_future();

	LogMessage(LOG_LEVEL_DEBUG, "Initiating connection attempt");
	for (const auto id : getVehicleIDs()) {
		auto t = std::thread(&ObjectControl::connectToObject, this,
							 std::ref(objects[id]),
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
	for (const auto id : getVehicleIDs()) {
		objects[id].disconnect();
	}
	objectListeners.clear();
}

void ObjectControl::disconnectObject(
		const uint32_t id) {
	objects[id].disconnect();
	objectListeners.erase(id);
}

void ObjectControl::uploadObjectConfiguration(
		const uint32_t id) {
	objects[id].sendSettings();
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

	LogMessage(LOG_LEVEL_DEBUG, "Starting heartbeat thread");
	while (stopRequest.wait_until(nextHeartbeat) == std::future_status::timeout) {
		nextHeartbeat += heartbeatPeriod;

		// Check time since MONR for all objects
		for (const auto& id : getVehicleIDs()) {
			if (objects[id].isConnected()) {
				auto diff = objects[id].getTimeSinceLastMonitor();
				if (diff > objects[id].getMaxAllowedMonitorPeriod()) {
					LogMessage(LOG_LEVEL_WARNING, "MONR timeout for object %u: %d ms > %d ms", id,
							   diff.count(), objects[id].getMaxAllowedMonitorPeriod().count());
					objects[id].disconnect();
					this->state->disconnectedFromObject(*this, id);
				}
			}
		}

		// Send heartbeat
		for (const auto& id : getVehicleIDs()) {
			try {
				if (objects[id].isConnected()) {
					objects[id].sendHeartbeat(this->state->asControlCenterStatus());
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_WARNING, e.what());
				objects[id].disconnect();
				this->state->disconnectedFromObject(*this, id);
			}
		}
	}
	LogMessage(LOG_LEVEL_INFO, "Heartbeat thread exiting");
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
	if (!objects[monr.first].getObjectConfig().getInjectionMap().targetIDs.empty()) {
		std::chrono::system_clock::time_point ts;
		auto secs = std::chrono::seconds(monr.second.timestamp.tv_sec);
		auto usecs = std::chrono::microseconds(monr.second.timestamp.tv_usec);
		ts += secs + usecs;
		auto osiGtData = buildOSILocalGroundTruth(monr);
		for (const auto& targetID : objects[monr.first].getObjectConfig().getInjectionMap().targetIDs) {
			if (objects[targetID].isOsiCompatible()) {
				objects[targetID].sendOsiData(osiGtData, objects[targetID].getProjString(), ts);
			}
		}
	}
}


void ObjectControl::startListeners() {
	LogMessage(LOG_LEVEL_DEBUG, "Starting listeners");
	objectListeners.clear();
	for (const auto& id : getVehicleIDs()) {
		objectListeners.try_emplace(id, this, &objects[id]);
	}
}

void ObjectControl::connectToObject(
		TestObject &obj,
		std::shared_future<void> &connStopReq) {
	constexpr int maxConnHeabs = 10;
	constexpr int maxConnMonrs = 10;
	try {
		if (!obj.isConnected()) {
			try {
				obj.establishConnection(connStopReq);
				obj.sendSettings();
			}
			catch (std::runtime_error& e) {
				LogMessage(LOG_LEVEL_ERROR, "Connection attempt for object %u failed: %s",
						   obj.getTransmitterID(), e.what());
				obj.disconnect();
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
						obj.sendHeartbeat(this->state->asControlCenterStatus());
						nextSendTime += heartbeatPeriod;
						objState = obj.getState(true, heartbeatPeriod);
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
						LogMessage(LOG_LEVEL_INFO, "Connected to armed object ID %u", obj.getTransmitterID());
						this->state->connectedToArmedObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
					case OBJECT_STATE_POSTRUN:
					case OBJECT_STATE_RUNNING:
						LogMessage(LOG_LEVEL_INFO, "Connected to running object ID %u", obj.getTransmitterID());
						this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_INIT:
						if (initializingMonrs-- > 0) {
							continue;
						}
						else {
							LogMessage(LOG_LEVEL_INFO, "Connected object %u in initializing state after connection", obj.getTransmitterID());
							this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						}
						break;
					case OBJECT_STATE_DISARMED:
						LogMessage(LOG_LEVEL_INFO, "Connected to disarmed object ID %u", obj.getTransmitterID());
						this->state->connectedToObject(*this, obj.getTransmitterID());
						break;
					default:
						LogMessage(LOG_LEVEL_INFO, "Connected to object %u in unknown state", obj.getTransmitterID());
						this->state->connectedToLiveObject(*this, obj.getTransmitterID());
					}
					break;
				}
			}
			catch (std::runtime_error &e) {
				LogMessage(LOG_LEVEL_ERROR, "Connection startup procedure failed for object %u: %s",
						   obj.getTransmitterID(), e.what());
				obj.disconnect();
				// TODO: connection failed event?
			}
		}
	}
	catch (std::invalid_argument &e) {
		LogMessage(LOG_LEVEL_ERROR, "Bad connection attempt for object %u: %s",
				   obj.getTransmitterID(), e.what());
		obj.disconnect();
		// TODO: connection failed event?
	}
};


void ObjectControl::armObjects() {
	for (auto& id : getVehicleIDs()) {
		objects[id].sendArm();
	}
}

void ObjectControl::disarmObjects() {
	for (auto& id : getVehicleIDs()) {
		try {
			objects[id].sendDisarm();
		}
		catch (std::invalid_argument& e) {
			LogMessage(LOG_LEVEL_ERROR, "Unable to disarm object %u: %s", id, e.what());
		}
	}
	this->state->allObjectsDisarmed(*this); // TODO add a check on object states as well
}

void ObjectControl::startObjects() {
	for (auto& id : getVehicleIDs()) {
		if (!objects[id].isStartingOnTrigger()) {
			objects[id].sendStart();
		}
	}
}

void ObjectControl::allClearObjects() {
	for (auto& id : getVehicleIDs()) {
		objects[id].sendAllClear();
	}
	this->state->allObjectsAbortDisarmed(*this); // TODO wait for all objects really are disarmed
}

bool ObjectControl::isAnyObjectIn(
		const ObjectStateType state) {
	return std::any_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

bool ObjectControl::areAllObjectsIn(
		const ObjectStateType state) {
	return std::all_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

bool ObjectControl::isAnyObjectIn(
		const std::set<ObjectStateType>& states) {
	return std::any_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,TestObject>& obj) {
		return states.find(obj.second.getState()) != states.end();
	});
}

bool ObjectControl::areAllObjectsIn(
		const std::set<ObjectStateType>& states) {
	return std::all_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,TestObject>& obj) {
		return states.find(obj.second.getState()) != states.end();
	});
}

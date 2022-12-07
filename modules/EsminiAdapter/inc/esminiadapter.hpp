#pragma once

#include "module.hpp"
#include "roschannel.hpp"
#include <unordered_map>
#include "util.h"
#include "esmini/esminiLib.hpp"
#include "trajectory.hpp"

/*!
 * \brief The EsminiAdapter class is a singleton class that 
 * 	handles the communication between the esmini simulator and Maestro
 */
class EsminiAdapter : public Module {
public:
	static inline std::string const moduleName = "esmini_adapter";
	static inline std::string const oscFileName = "atos_scenario.xosc";
	static inline std::string oscFilePath;
	static int initializeModule(const LOG_LEVEL logLevel);
	EsminiAdapter(EsminiAdapter const&) = delete;
    EsminiAdapter& operator=(EsminiAdapter const&) = delete;
	static std::shared_ptr<EsminiAdapter> instance();

private:
	EsminiAdapter();
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;
	void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id) override;
	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onStartMessage(const ROSChannels::Start::message_type::SharedPtr) override;
	void onExitMessage(const ROSChannels::Start::message_type::SharedPtr) override;
	static void reportObjectPosition(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id);
	static void onEsminiStoryBoardStateChange(const char* name, int type, int state);
	static void onEsminiConditionTriggered(const char* name, double timestamp);
	static void InitializeEsmini(std::string& oscFilePath);
	static void getObjectStates(const std::string& oscFilePath, double timeStep, double endTime, std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states);
	static maestro::Trajectory getTrajectory(uint32_t,std::vector<SE_ScenarioObjectState>& states);
	static std::map<uint32_t,maestro::Trajectory> extractTrajectories(const std::string& oscFilePath, double timeStep, double endTime, std::map<uint32_t,maestro::Trajectory>& idToTraj);
	
	static std::shared_ptr<EsminiAdapter> me;
	static std::unordered_map<int, int> objectIdToIndex;

};

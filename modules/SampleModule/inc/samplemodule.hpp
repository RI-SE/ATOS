#pragma once

#include <thread>
#include "module.hpp"
#include "server.hpp"

/*!
 * \brief The SampleModule is a ros2 node that demonstrates how to use the Module class 
 */
class SampleModule : public Module{
public:
	static inline std::string const moduleName = "sample_module";
	SampleModule();
	std::vector<std::uint32_t> getObjectIds();
	bool getAborting() { return aborting_; }

private:
	ROSChannels::Init::Sub initSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::AllClear::Sub allClearSub;

	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;

	std::vector<std::uint32_t> objectIds;
	bool aborting_ = false;
};

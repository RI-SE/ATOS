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
	~SampleModule();

private:
	static inline const int TCPPort = 1337;

	void tcpSocketProcedure();
	ROSChannels::Init::Sub initSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::AllClear::Sub allClearSub;

	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;

	std::unique_ptr<std::thread> tcpThread;
	TCPServer tcpServer;
	bool quit = false;
};

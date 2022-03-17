#pragma once

#include <thread>
#include "module.hpp"
#include "tcphandler.hpp"
#include "server.hpp"
#include "maestro_interfaces/msg/control_signal_percentage.hpp"

class DirectControl : public Module {
public:
	static inline std::string const moduleName = "direct_control";
	DirectControl();
	int initializeModule(const LOG_LEVEL logLevel);
	void startThreads();
	void joinThreads();

private:
	static inline const int TCPPort = 53260;
	static inline const int UDPPort = 53995;

	void readTCPSocketData();
	void readUDPSocketData();
	void handleISOMessage(std::vector<char>& byteData, size_t receivedBytes);
	size_t handleRDCAMessage(std::vector<char>& byteData);
	size_t handleUnknownMessage(std::vector<char>& byteData);
	ROSChannels::ControlSignalPercentage::Pub controlSignalPercentagePub;

	void onAbortMessage(const Empty::SharedPtr) override;
	void onAllClearMessage(const Empty::SharedPtr) override;

	std::unique_ptr<std::thread> receiveThread;
	std::unique_ptr<std::thread> receiveThreadUDP;
	volatile bool quit = false;
	TCPHandler tcpHandler;
	UDPServer udpServer;
};

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <thread>
#include "module.hpp"
#include "tcphandler.hpp"
#include "server.hpp"
#include "atos_interfaces/msg/control_signal_percentage.hpp"

class DirectControl : public Module {
public:
	static inline std::string const moduleName = "direct_control";
	DirectControl();
	int initializeModule();
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
	ROSChannels::ControlSignal::Pub controlSignalPub;

	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;

	std::unique_ptr<std::thread> receiveThread;
	std::unique_ptr<std::thread> receiveThreadUDP;
	volatile bool quit = false;
	TCPHandler tcpHandler;
	UDPServer udpServer;
};

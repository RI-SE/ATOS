#pragma once

#include <thread>
#include "module.hpp"
#include "tcphandler.hpp"
#include "server.hpp"

class DirectControl : public Module {
public:
	DirectControl();
	void readMessageBus();
	void readSocketData();
	void readUDPSocketData();
	void handleISOMessage(std::vector<char>& byteData, size_t receivedBytes);
	size_t handleRDCAMessage(std::vector<char>& byteData);
	size_t handleUnknownMessage(std::vector<char>& byteData);
	int run();
	void exit();

	static constexpr unsigned int commandPort = 53260;
	static inline const int UDPPort = 53995;
private:
	void onAbortMessage(const Empty::SharedPtr) override;
	void onAllClearMessage(const Empty::SharedPtr) override;
	volatile bool quit = false;
	TCPHandler tcpHandler;
	UDPServer udpServer;
};

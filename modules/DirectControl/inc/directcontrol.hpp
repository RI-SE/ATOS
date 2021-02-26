#pragma once

#include <thread>
#include "tcphandler.hpp"

class DirectControl {
public:
	DirectControl();
	void readMessageBus();
	void readSocketData();
	void handleISOMessage(std::vector<char>& byteData, size_t receivedBytes);
	size_t handleRDCAMessage(std::vector<char>& byteData);
	size_t handleUnknownMessage(std::vector<char>& byteData);
	int run();
	void exit();

	static constexpr unsigned int commandPort = 53260;
private:
	volatile bool quit = false;
	TCPHandler tcpHandler;
};

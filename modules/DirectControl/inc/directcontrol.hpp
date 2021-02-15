#pragma once

#include <thread>
#include "tcphandler.hpp"

class DirectControl {
public:
	DirectControl();
	void readMessageBus();
	void readSocketData();
	int run();
	void exit();

	static constexpr unsigned int commandPort = 53260;
private:
	volatile bool quit = false;
};

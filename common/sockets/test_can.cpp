#include "canhandler.hpp"
#include <cassert>
#include <iostream>

int main(int argc, char** argv) {
	CANHandler ch;
	std::string interface = "slcan0";
	std::cout << "Connecting to interface " << interface << std::endl;
	assert(ch.connectTo(interface) >= 0);

	// TODO test receive functionality

	can_frame frame;
	frame.can_id = 0x123;
	frame.data[0] = 0x0B;
	frame.data[1] = 0x0E;
	frame.data[2] = 0x0E;
	frame.data[3] = 0x0F;
	frame.can_dlc = 4;

	std::cout << "Sending frame " << frame.can_id << " on CAN" << std::endl;
	ssize_t bytesSent = ch.transmit(frame);
	assert(bytesSent > 0); // TODO exact byte count
	std::cout << "Sent " << bytesSent << " bytes" << std::endl;

	frame.can_id = 0x100;
	frame.data[0] = 0x0D;
	frame.data[1] = 0x0E;
	frame.data[2] = 0x0A;
	frame.data[3] = 0x0D;
	frame.data[4] = 0x0B;
	frame.data[5] = 0x0E;
	frame.data[6] = 0x0E;
	frame.data[7] = 0x0F;
	frame.can_dlc = 8;

	std::cout << "Sending frame " << frame.can_id << " on CAN" << std::endl;
	bytesSent = ch.transmit(frame);
	assert(bytesSent > 0); // TODO exact byte count
	std::cout << "Sent " << bytesSent << " bytes" << std::endl;
	std::cout << "CAN handler test successful" << std::endl;
	return 0;

}

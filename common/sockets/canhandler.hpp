#ifndef CANHANDLER_HPP
#define CANHANDLER_HPP
#include <string>
#include <linux/can.h>

class CANHandler {
public:
	CANHandler(const bool blocking = true);
	void setBlocking(void) { this->blocking = true; }
	void setNonblocking(void) { this->blocking = false; }
	bool isBlocking() const { return this->blocking; }
	int connectTo(const std::string& interface);
	ssize_t receive(can_frame &frame);
	ssize_t transmit(const can_frame &frame);
private:
	int sockfd = 0;
	struct sockaddr_can addr = {0};
	bool blocking = true;
};


#endif

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "canhandler.hpp"
#include <iostream>
#include <cstring>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>


CANHandler::CANHandler(const bool blocking) {
	this->blocking = blocking;
}

int CANHandler::connectTo(
		const std::string &interface) {

	struct ifreq ifr;
	can_frame frame;
	ssize_t bytesRead = 0;

	if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return -1;
	}

	std::strcpy(ifr.ifr_name, interface.c_str());
	ioctl(sockfd, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	std::cout << "Binding CAN handler to interface " << interface << std::endl;
	if (bind(sockfd, reinterpret_cast<struct sockaddr *>(&addr),
			 sizeof (addr))) {
		perror("bind");
		return -1;
	}

	bytesRead = recv(sockfd, &frame, sizeof (frame), MSG_DONTWAIT);
	if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		perror("recv");
		close(sockfd);
		return -1;
	}

	return 0;
}

ssize_t CANHandler::receive(can_frame &frame) {
	ssize_t bytesRead = 0;
	bytesRead = recv(sockfd, &frame, sizeof (frame),
						blocking ? 0 : MSG_DONTWAIT);
	if (bytesRead < 0) {
		if (!blocking && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			bytesRead = 0;
		}
		else {
			perror("recv");
			close(sockfd);
		}
	}
	return bytesRead;
}

ssize_t CANHandler::transmit(const can_frame &frame){
	ssize_t bytesSent = 0;
	bytesSent = send(sockfd, &frame, sizeof (struct can_frame), 
						blocking ? 0 : MSG_DONTWAIT);
	if (bytesSent < 0) {
		if (!blocking && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			bytesSent = 0;
		}
		else {
			perror("send");
			close(sockfd);
		}
	}
	return bytesSent;
}

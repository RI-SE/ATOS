/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <vector>
#include <chrono>
#include <future>
#include <netinet/in.h>
#include "loggable.hpp"
#include "iso22133.h"
#include "trajectory.hpp"
#include "roschannels/controlsignalchannel.hpp"

struct MonitorMessage : std::pair<uint32_t,ObjectMonitorType> {};

class Channel : public Loggable
{
public:
	Channel(const size_t bufferLength, const int type, rclcpp::Logger log)
		: channelType(type),
		  transmitBuffer(bufferLength, 0),
		  receiveBuffer(bufferLength, 0),
		  Loggable(log)
	{}
	Channel(int type, rclcpp::Logger log) : Channel(1024, type, log) {}
	struct sockaddr_in addr = {};
	int socket = -1;
	int channelType = 0; //!< SOCK_STREAM or SOCK_DGRAM
	std::vector<char> transmitBuffer;
	std::vector<char> receiveBuffer;

	ISOMessageID pendingMessageType(bool awaitNext = false);
	std::string remoteIP() const;
	bool isValid() const { return socket != -1; }
	void connect(std::shared_future<void> stopRequest,
				 const std::chrono::milliseconds retryPeriod);
	void disconnect();

	friend Channel& operator<<(Channel&,const HeabMessageDataType&);
	friend Channel& operator<<(Channel&,const ObjectSettingsType&);
	friend Channel& operator<<(Channel&,const ATOS::Trajectory&);
	friend Channel& operator<<(Channel&,const ObjectCommandType&);
	friend Channel& operator<<(Channel&,const StartMessageType&);
	friend Channel& operator<<(Channel&,const std::vector<char>&);
	friend Channel& operator<<(Channel&,const ROSChannels::ControlSignal::message_type::SharedPtr csp);

	friend Channel& operator>>(Channel&,MonitorMessage&);
	friend Channel& operator>>(Channel&,ObjectPropertiesType&);
	friend Channel& operator>>(Channel&,GeneralResponseMessageType&);
};
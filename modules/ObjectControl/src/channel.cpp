/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "channel.hpp"
#include "iso22133.h"
#include <cstring>
#include "atosTime.h"
#include "header.h"

using namespace ROSChannels;

Channel& operator<<(Channel& chnl, const HeabMessageDataType& heartbeat) {
	MessageHeaderType header;
	auto nBytes = encodeHEABMessage(chnl.populateHeaderType(&header), &heartbeat.dataTimestamp, heartbeat.controlCenterStatus,
									chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode HEAB message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send HEAB: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ObjectSettingsType& settings) {
	MessageHeaderType header;
	auto nBytes = encodeOSEMMessage(chnl.populateHeaderType(&header), &settings, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode OSEM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send OSEM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ATOS::Trajectory& traj) {
	ssize_t nBytes;

	// TRAJ header
	MessageHeaderType header;
	nBytes = encodeTRAJMessageHeader(chnl.populateHeaderType(&header),
				traj.id, TRAJECTORY_INFO_RELATIVE_TO_ORIGIN, traj.name.c_str(),traj.name.length(),
				static_cast<uint32_t>(traj.points.size()), chnl.transmitBuffer.data(),
				chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send TRAJ message header: ") + strerror(errno));
	}

	// TRAJ points
	for (const auto& pt : traj.points) {
		struct timeval relTime;
		CartesianPosition pos = pt.getISOPosition();
		SpeedType spd = pt.getISOVelocity();
		AccelerationType acc = pt.getISOAcceleration();

		relTime = to_timeval(pt.getTime());

		nBytes = encodeTRAJMessagePoint(&relTime, pos, spd, acc, static_cast<float>(pt.getCurvature()),
										chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
		if (nBytes < 0) {
			// TODO what to do here?
			throw std::invalid_argument(std::string("Failed to encode TRAJ message point: ") + strerror(errno));
		}
		nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);

		if (nBytes < 0) {
			// TODO what to do here?
			throw std::runtime_error(std::string("Failed to send TRAJ message point: ") + strerror(errno));
		}
	}

	// TRAJ footer
	nBytes = encodeTRAJMessageFooter(chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message footer: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send TRAJ message footer: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ObjectCommandType& cmd) {
	MessageHeaderType header;
	auto nBytes = encodeOSTMMessage(chnl.populateHeaderType(&header), cmd, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode OSTM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send OSTM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const StartMessageType& strt) {
	MessageHeaderType header;
	auto nBytes = encodeSTRTMessage(chnl.populateHeaderType(&header), &strt, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode STRT message: ") + strerror(errno));
	}

	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send STRT: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator>>(Channel& chnl, MonitorMessage& monitor) {
	if (chnl.pendingMessageType() == MESSAGE_ID_MONR) {
		struct timeval tv;
		TimeSetToCurrentSystemTime(&tv);
		HeaderType header;
		decodeISOHeader(chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), &header, false);
		monitor.first = header.transmitterID;
		auto nBytes = decodeMONRMessage(chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), tv,
										&monitor.second, false);
		if (nBytes < 0) {
			throw std::invalid_argument("Failed to decode MONR message");
		}
		else {
			nBytes = recv(chnl.socket, chnl.receiveBuffer.data(), static_cast<size_t>(nBytes), 0);
			if (nBytes <= 0) {
				throw std::runtime_error("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}

Channel& operator>>(Channel& chnl, ObjectPropertiesType& prop) {
	if (chnl.pendingMessageType() == MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO) {
		auto nBytes = decodeOPROMessage(&prop, chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), false);
		if (nBytes < 0) {
			throw std::invalid_argument(strerror(errno));
		}
		else {
			nBytes = recv(chnl.socket, chnl.receiveBuffer.data(), static_cast<size_t>(nBytes), 0);
			if (nBytes <= 0) {
				throw std::runtime_error("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}

Channel& operator>>(Channel& chnl, GeneralResponseMessageType& grem) {
	if (chnl.pendingMessageType() == MESSAGE_ID_GREM) {
		auto nBytes = decodeGREMMessage(chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), &grem, false);
		if (nBytes < 0) {
			throw std::invalid_argument(strerror(errno));
		}
		else {
			nBytes = recv(chnl.socket, chnl.receiveBuffer.data(), static_cast<size_t>(nBytes), 0);
			if (nBytes <= 0) {
				throw std::runtime_error("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ControlSignal::message_type::SharedPtr csp) {
	RemoteControlManoeuvreMessageType rcmm;
	rcmm.command = MANOEUVRE_NONE;
	rcmm.isThrottleManoeuvreValid = true;
	rcmm.isBrakeManoeuvreValid = true;
	rcmm.isSteeringManoeuvreValid = true;
	rcmm.throttleUnit = ISO_UNIT_TYPE_THROTTLE_PERCENTAGE;
	rcmm.brakeUnit = ISO_UNIT_TYPE_BRAKE_PERCENTAGE;
	rcmm.steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
	rcmm.throttleManoeuvre.pct = csp->throttle;
	rcmm.brakeManoeuvre.pct = csp->brake;
	rcmm.steeringManoeuvre.pct = csp->steering_angle;

	MessageHeaderType header;
	auto nBytes = encodeRCMMMessage(chnl.populateHeaderType(&header),&rcmm, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode RCM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send RCM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const std::vector<char>& data) {
	auto nBytes = send(chnl.socket, data.data(), data.size(), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send raw data: ") + strerror(errno));
	}
	return chnl;
}

std::string Channel::remoteIP() const {
	char ipString[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->addr.sin_addr, ipString, sizeof (ipString));
	return std::string(ipString);
}

ISOMessageID Channel::pendingMessageType(bool awaitNext) {
	auto result = recv(this->socket, this->receiveBuffer.data(), this->receiveBuffer.size(), (awaitNext ? 0 : MSG_DONTWAIT) | MSG_PEEK);
	if (result < 0 && !awaitNext && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		return MESSAGE_ID_INVALID;
	}
	else if (result < 0) {
		throw std::runtime_error(std::string("Failed to check pending message type (recv: ")
									 + strerror(errno) + ")");
	}
	else if (result == 0) {
		throw std::runtime_error("Connection reset by peer");
	}
	else {
		ISOMessageID retval = getISOMessageType(this->receiveBuffer.data(), this->receiveBuffer.size(), false);
		if (retval == MESSAGE_ID_INVALID) {
			throw std::runtime_error("Non-ISO message received from " + this->remoteIP());
		}
		return retval;
	}
}

MessageHeaderType *Channel::populateHeaderType(MessageHeaderType *header) {
	memset(header, 0, sizeof (MessageHeaderType));
	header->transmitterID = this->transmitterId;
	header->receiverID = this->objectId;
	header->messageCounter = this->sentMessageCounter++;
	return header;
}

void Channel::connect(
		std::shared_future<void> stopRequest,
		const std::chrono::milliseconds retryPeriod) {
	char ipString[INET_ADDRSTRLEN];
	std::stringstream errMsg;

	if (inet_ntop(AF_INET, &this->addr.sin_addr, ipString, sizeof (ipString))
			== nullptr) {
		errMsg << "inet_ntop: " << strerror(errno);
		throw std::invalid_argument(errMsg.str());
	}

	if (this->addr.sin_addr.s_addr == 0) {
		errMsg << "Invalid connection IP specified: " << ipString;
		throw std::invalid_argument(errMsg.str());
	}

	std::string type = "???";
	if (this->channelType == SOCK_STREAM) {
		type = "TCP";
	}
	else if (this->channelType == SOCK_DGRAM) {
		type = "UDP";
	}

	this->socket = ::socket(AF_INET, this->channelType, 0);
	if (this->socket < 0) {
		errMsg << "Failed to open " << type << " socket: " << strerror(errno);
		this->disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Begin connection attempt
	RCLCPP_INFO(get_logger(), "Attempting %s connection to %s:%u", type.c_str(), ipString,
			   ntohs(this->addr.sin_port));

	while (true) {
		if (::connect(this->socket,
					reinterpret_cast<struct sockaddr *>(&this->addr),
					sizeof (this->addr)) == 0) {
			break;
		}
		else {
			RCLCPP_ERROR(get_logger(), "Failed %s connection attempt to %s:%u, retrying in %.3f s ...",
					   type.c_str(), ipString, ntohs(this->addr.sin_port), retryPeriod.count() / 1000.0);
			if (stopRequest.wait_for(retryPeriod)
					!= std::future_status::timeout) {
				errMsg << "Connection attempt interrupted";
				throw std::runtime_error(errMsg.str());
			}
		}
	}
}

void Channel::disconnect() {
	if (this->socket != -1) {
		if (shutdown(this->socket, SHUT_RDWR) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket shutdown: %s", strerror(errno));
		}
		if (close(this->socket) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket close: %s", strerror(errno));
		}
		this->socket = -1;
	}
}
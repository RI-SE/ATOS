/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file osi_handler.cpp
 * @author Albin Nykvist
 * @brief  Osi handler class for encoding/decoding osi messages
 * @date 2021-05-25
 *
 * @copyright Copyright (c) AstaZero 2021
 *
 */

#include "osi_handler.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <algorithm>

void OsiHandler::decodeSdMessage(
	const std::vector<char>& msg,
	const int msgSize,
	const bool debug) {

	std::string data(msg.begin(),msg.begin()+msgSize);

	if (Sd.ParseFromString(data)) {
		
		if (debug) {
			using std::cout, std::endl;
			cout << "Decoder debug:" << endl;
			cout << Sd.DebugString() << endl;
		}
	}
	else {
		throw std::invalid_argument("Couldn't decode message!");
	}
}


void OsiHandler::decodeSvGtMessage(
		const std::vector<char>& msg,
		const int msgSize,
		std::vector<OsiHandler::GlobalObjectGroundTruth_t>& retval,
		std::string& projStr,
		const bool debug) {

	std::string data(msg.begin(),msg.begin()+msgSize);

	if (Sv.ParseFromString(data)) {
		if (Sv.has_global_ground_truth()) {
			int seconds = Sv.global_ground_truth().timestamp().seconds();
			int nanos = Sv.global_ground_truth().timestamp().nanos();
			float timestamp = seconds + nanos / pow(10,9);
			std::cout << "Simulation timestamp: " << timestamp << std::endl;

			int no_of_mov_obj = Sv.global_ground_truth().moving_object_size();
			for (int i=0; i< no_of_mov_obj; i++) {
				osi3::MovingObject obj = Sv.global_ground_truth().moving_object(i);
				OsiHandler::GlobalObjectGroundTruth_t ret;
				ret.id = obj.id().value();
				ret.pos_m.x = obj.base().position().x();
				ret.pos_m.y = obj.base().position().y();
				ret.pos_m.z = obj.base().position().z();
				ret.vel_m_s.x = obj.base().velocity().x();
				ret.vel_m_s.y = obj.base().velocity().y();
				ret.vel_m_s.z = obj.base().velocity().z();
				ret.acc_m_s2.x = obj.base().acceleration().x();
				ret.acc_m_s2.y = obj.base().acceleration().y();
				ret.acc_m_s2.z = obj.base().acceleration().z();
				ret.orientation_rad.yaw = obj.base().orientation().yaw();
				ret.orientation_rad.pitch = obj.base().orientation().pitch();
				ret.orientation_rad.roll = obj.base().orientation().roll();

				//Check if ID exists in objects ##TEMP
				if(std::any_of(retval.begin(),retval.end(), [&](const OsiHandler::GlobalObjectGroundTruth_t& elem){
					return elem.id == ret.id;
				}))
				{
					continue;
				}

				retval.push_back(ret);
				projStr = Sv.global_ground_truth().proj_string();
			}
			if (debug) {
				using std::cout, std::endl;
				cout << "Decoder debug:" << endl;
				cout << Sv.DebugString() << endl;
			}
		}
		else {
			std::cout << "Message has no groundtruth content" << std::endl;
		}
	}
	else {
		throw std::invalid_argument("Couldn't decode message!");
	}
}

std::string OsiHandler::encodeSvGtMessage(
		const std::vector<LocalObjectGroundTruth_t>& data,
		const std::chrono::system_clock::time_point& timestamp,
		const std::string& projectionString, const bool debug) {
	auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp).time_since_epoch().count();
	auto secs = nanos/1000000000;
	nanos = nanos - secs*1000000000;

	Sv.Clear();

	osi3::GroundTruth *groundTruth = Sv.mutable_global_ground_truth();
	groundTruth->set_proj_string(projectionString);
	groundTruth->mutable_timestamp()->set_seconds(secs);
	groundTruth->mutable_timestamp()->set_nanos(static_cast<unsigned int>(nanos));

	for (const auto& elem : data) {
		using Eigen::AngleAxisd, Eigen::Vector3d;
		osi3::MovingObject *movingObject = groundTruth->add_moving_object();
		movingObject->mutable_id()->set_value(elem.id);
		auto objOrientation = movingObject->mutable_base()->mutable_orientation();
		auto objPosition = movingObject->mutable_base()->mutable_position();
		auto objVelocity = movingObject->mutable_base()->mutable_velocity();
		auto objAcceleration = movingObject->mutable_base()->mutable_acceleration();

		objPosition->set_x(elem.pos_m.x);
		objPosition->set_y(elem.pos_m.y);
		objPosition->set_z(elem.pos_m.z);
		objOrientation->set_yaw(elem.orientation_rad.yaw);
		objOrientation->set_pitch(elem.orientation_rad.pitch);
		objOrientation->set_roll(elem.orientation_rad.roll);

		auto rot = AngleAxisd(elem.orientation_rad.roll, Vector3d::UnitX())
		  * AngleAxisd(elem.orientation_rad.pitch,  Vector3d::UnitY())
		  * AngleAxisd(elem.orientation_rad.yaw, Vector3d::UnitZ());

		Vector3d velObjFrame(elem.vel_m_s.lon,elem.vel_m_s.lat,elem.vel_m_s.up);
		Vector3d velWorldFrame = rot.inverse() * velObjFrame;

		objVelocity->set_x(velWorldFrame.x());
		objVelocity->set_y(velWorldFrame.y());
		objVelocity->set_z(velWorldFrame.z());

		Vector3d accObjFrame(elem.acc_m_s2.lon,elem.acc_m_s2.lat,elem.acc_m_s2.up);
		Vector3d accWorldFrame = rot.inverse() * accObjFrame;

		objAcceleration->set_x(accWorldFrame.x());
		objAcceleration->set_y(accWorldFrame.y());
		objAcceleration->set_z(accWorldFrame.z());
	}

	if (debug) {
		using std::cout, std::endl;
		cout << "Encoder debug:" << std::endl;
		cout << Sv.DebugString() << std::endl;
	}
	return Sv.SerializeAsString();
}

std::string OsiHandler::encodeSvGtMessage(
		const std::vector<GlobalObjectGroundTruth_t> &data,
		const std::chrono::system_clock::time_point &timestamp,
		const std::string &projectionString, const bool debug) {

	auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp).time_since_epoch().count();
	auto secs = nanos/1000000000;
	nanos = nanos - secs*1000000000;

	Sv.Clear();

	osi3::GroundTruth *groundTruth = Sv.mutable_global_ground_truth();
	groundTruth->set_proj_string(projectionString);
	groundTruth->mutable_timestamp()->set_seconds(secs);
	groundTruth->mutable_timestamp()->set_nanos(static_cast<unsigned int>(nanos));

	for (const auto& elem : data) {
		osi3::MovingObject *movingObject = groundTruth->add_moving_object();
		movingObject->mutable_id()->set_value(elem.id);
		movingObject->mutable_base()->mutable_position()->set_x(elem.pos_m.x);
		movingObject->mutable_base()->mutable_position()->set_y(elem.pos_m.y);
		movingObject->mutable_base()->mutable_position()->set_z(elem.pos_m.z);
		movingObject->mutable_base()->mutable_velocity()->set_x(elem.vel_m_s.x);
		movingObject->mutable_base()->mutable_velocity()->set_y(elem.vel_m_s.y);
		movingObject->mutable_base()->mutable_velocity()->set_z(elem.vel_m_s.z);
		movingObject->mutable_base()->mutable_acceleration()->set_x(elem.acc_m_s2.x);
		movingObject->mutable_base()->mutable_acceleration()->set_y(elem.acc_m_s2.y);
		movingObject->mutable_base()->mutable_acceleration()->set_z(elem.acc_m_s2.z);
		movingObject->mutable_base()->mutable_orientation()->set_pitch(elem.orientation_rad.pitch);
		movingObject->mutable_base()->mutable_orientation()->set_roll(elem.orientation_rad.roll);
		movingObject->mutable_base()->mutable_orientation()->set_yaw(elem.orientation_rad.yaw);
	}

	if (debug) {
		using std::cout, std::endl;
		cout << "Encoder debug:" << endl;
		cout << Sv.DebugString() << endl;
	}
	return Sv.SerializeAsString();
}

std::string OsiHandler::encodeSvGtMessage(
		const LocalObjectGroundTruth_t& data,
		const std::chrono::system_clock::time_point& timestamp,
		const std::string& projectionString, const bool debug) {
	return this->encodeSvGtMessage(std::vector<LocalObjectGroundTruth_t>({data}), timestamp, projectionString, debug);
}

std::string OsiHandler::encodeSvGtMessage(
		const GlobalObjectGroundTruth_t& data,
		const std::chrono::system_clock::time_point& timestamp,
		const std::string& projectionString, const bool debug) {
	return this->encodeSvGtMessage(std::vector<GlobalObjectGroundTruth_t>({data}), timestamp, projectionString, debug);
}

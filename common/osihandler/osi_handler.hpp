/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * @file osi_handler.h
 * @author Albin Nykvist 
 * @brief  Osi handler class for encoding/decoding osi messages 
 * @date 2021-05-25
 * 
 * @copyright Copyright (c) AstaZero 2021
 * 
 */

#pragma once
#include <chrono>
#include "osi3/osi_sensorview.pb.h"
#include "osi3/osi_sensordata.pb.h"

class OsiHandler
{
public:
	/*!
	 * \struct GlobalObjectGroundTruth_t
	 * \brief The GlobalObjectGroundTruth_t struct represents data for a single object in an earth-fixed frame.
	 * \var GlobalObjectGroundTruth_t::id ID of the object.
	 * \var GlobalObjectGroundTruth_t::pos_m Position of the object
	 * \var GlobalObjectGroundTruth_t::vel_m_s Velocity of the object
	 * \var GlobalObjectGroundTruth_t::acc_m_s2 Acceleration of the object
	 */
	typedef struct {
		unsigned long id;
		/*!
		 * \struct GlobalObjectGroundTruth_t::pos_m
		 * \brief The GlobalObjectGroundTruth_t::pos_m struct represents a position in an earth-fixed frame
		 * \var GlobalObjectGroundTruth_t::pos_m::x x position, in m
		 * \var GlobalObjectGroundTruth_t::pos_m::y y position, in m
		 * \var GlobalObjectGroundTruth_t::pos_m::y z position, in m
		 */
		struct {
			double x;
			double y;
			double z;
		} pos_m;

		/*!
		 * \struct GlobalObjectGroundTruth_t::vel_m_s
		 * \brief The GlobalObjectGroundTruth_t::vel_m_s struct represents a velocity in an earth-fixed frame
		 * \var GlobalObjectGroundTruth_t::vel_m_s::x Speed along the x axis, in m/s
		 * \var GlobalObjectGroundTruth_t::vel_m_s::y Speed along the y axis, in m/s
		 * \var GlobalObjectGroundTruth_t::vel_m_s::z Speed along the z axis, in m/s
		 */
		struct {
			double x;
			double y;
			double z;
		} vel_m_s;

		/*!
		 * \struct GlobalObjectGroundTruth_t::acc_m_s2
		 * \brief The GlobalObjectGroundTruth_t::acc_m_s2 struct represents an acceleration in an earth-fixed frame
		 * \var GlobalObjectGroundTruth_t::acc_m_s2::x Acceleration along the x axis, in m/s²
		 * \var GlobalObjectGroundTruth_t::acc_m_s2::y Acceleration along the y axis, in m/s²
		 * \var GlobalObjectGroundTruth_t::acc_m_s2::z Acceleration along the z axis, in m/s²
		 */
		struct {
			double x;
			double y;
			double z;
		} acc_m_s2;

		/*!
		 * \struct GlobalObjectGroundTruth_t::orientation_rad
		 * \brief The GlobalObjectGroundTruth_t::orientation_rad struct represents an orientation in
		 *			the 3d space relative to an earth-fixed coordinate system. The representation is
		 *			in Euler angles and the order is yaw, followed by pitch and then roll.
		 * \var GlobalObjectGroundTruth_t::orientation_rad::yaw Yaw in radians around earth-fixed Z.
		 * \var GlobalObjectGroundTruth_t::orientation_rad::pitch Pitch in radians around new Y.
		 * \var GlobalObjectGroundTruth_t::orientation_rad::roll Roll in radians around new X.
		 * */
		struct {
			double yaw;
			double pitch;
			double roll;
		} orientation_rad;
	} GlobalObjectGroundTruth_t;

	/*!
	 * \struct LocalObjectGroundTruth_t
	 * \brief The LocalObjectGroundTruth_t struct represents data for a single object with acceleration and
	 *			velocity in the object-local frame.
	 * \var LocalObjectGroundTruth_t::id ID of the object.
	 * \var LocalObjectGroundTruth_t::pos_m Position of the object
	 * \var LocalObjectGroundTruth_t::vel_m_s Velocity of the object
	 * \var LocalObjectGroundTruth_t::acc_m_s2 Acceleration of the object
	 */
	typedef struct {
		unsigned long id;

		/*!
		 * \struct LocalObjectGroundTruth_t::pos_m
		 * \brief The LocalObjectGroundTruth_t::pos_m struct represents a position in an earth-fixed frame
		 * \var LocalObjectGroundTruth_t::pos_m::x x position, in m
		 * \var LocalObjectGroundTruth_t::pos_m::y y position, in m
		 * \var LocalObjectGroundTruth_t::pos_m::y z position, in m
		 */
		struct {
			double x;
			double y;
			double z;
		} pos_m;

		/*!
		 * \struct LocalObjectGroundTruth_t::vel_m_s
		 * \brief The LocalObjectGroundTruth_t::vel_m_s struct represents a velocity in the object-local frame
		 * \var LocalObjectGroundTruth_t::vel_m_s::lon Longitudinal speed in m/s², positive forward
		 * \var LocalObjectGroundTruth_t::vel_m_s::lat Lateral speed in m/s², positive left
		 * \var LocalObjectGroundTruth_t::vel_m_s::up Upward speed in m/s², positive up
		 */
		struct {
			double lon;
			double lat;
			double up;
		} vel_m_s;

		/*!
		 * \struct LocalObjectGroundTruth_t::acc_m_s2
		 * \brief The LocalObjectGroundTruth_t::acc_m_s2 struct represents an acceleration in the
		 *			object-local frame
		 * \var LocalObjectGroundTruth_t::acc_m_s2::lon Longitudinal acceleration in m/s², positive forward
		 * \var LocalObjectGroundTruth_t::acc_m_s2::lat Lateral acceleration in m/s², positive left
		 * \var LocalObjectGroundTruth_t::acc_m_s2::up Upward acceleration in m/s², positive up
		 */
		struct {
			double lon;
			double lat;
			double up;
		} acc_m_s2;

		/*!
		 * \struct LocalObjectGroundTruth_t::orientation_rad
		 * \brief The LocalObjectGroundTruth_t::orientation_rad struct represents an orientation in
		 *			the 3d space relative to an earth-fixed coordinate system. The representation is
		 *			in Euler angles and the order is yaw, followed by pitch and then roll.
		 * \var LocalObjectGroundTruth_t::orientation_rad::yaw Yaw in radians around earth-fixed Z.
		 * \var LocalObjectGroundTruth_t::orientation_rad::pitch Pitch in radians around new Y.
		 * \var LocalObjectGroundTruth_t::orientation_rad::roll Roll in radians around new X.
		 * */
		struct {
			double yaw;
			double pitch;
			double roll;
		} orientation_rad;
	} LocalObjectGroundTruth_t;

    /*!
     * \brief Decode groundtruth content of Sensorview osi message. 
     * \param msg Recieved serialized OSI message. 
     * \param msgSize Size of recieved OSI message. 
     * \param debug Debug flag for printing message content.
     */
	void decodeSvGtMessage(const std::vector<char>& msg, const int msgSize, std::vector<GlobalObjectGroundTruth_t>& retval, std::string& projStr, const bool debug);
	std::string encodeSvGtMessage(const std::vector<GlobalObjectGroundTruth_t>& data,
						   const std::chrono::system_clock::time_point& timestamp,
						   const std::string& projectionString, const bool debug);

	std::string encodeSvGtMessage(const GlobalObjectGroundTruth_t& data,
						   const std::chrono::system_clock::time_point& timestamp,
						   const std::string& projectionString, const bool debug);

	std::string encodeSvGtMessage(const std::vector<LocalObjectGroundTruth_t>& data,
						   const std::chrono::system_clock::time_point& timestamp,
						   const std::string& projectionString, const bool debug);

	std::string encodeSvGtMessage(const LocalObjectGroundTruth_t& data,
						   const std::chrono::system_clock::time_point& timestamp,
						   const std::string& projectionString, const bool debug);

	/*!
     * \brief Decode content of SensorData osi message. 
     * \param msg Recieved serialized OSI message. 
     * \param msgSize Size of recieved OSI message. 
     * \param debug Debug flag for printing message content.
     */
	void decodeSdMessage(const std::vector<char>& msg, const int msgSize, const bool debug);

    // Public Sensorview message object. Calling decodeSvGtMessage will update the content. 
	osi3::SensorView Sv; // Use the internal methods to access the data. For example see the debug section of decodeSvGtMessage
	// Public Sensordata message object. Calling decodeSdMessage will update the content. 
	osi3::SensorData Sd; // Use the internal methods to access the data. For example see the debug section of decodeSdMessage
    
};

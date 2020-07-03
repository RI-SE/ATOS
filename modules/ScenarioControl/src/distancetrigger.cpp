#include <sstream>
#include <iostream>
#include <algorithm>
#include "maestroTime.h"
#include "distancetrigger.h"
#include "util.h"

constexpr CartesianPosition DistanceTrigger::defaultReferencePoint;

DistanceTrigger::DistanceTrigger(Trigger::TriggerID_t triggerID) : BooleanTrigger(triggerID, Trigger::TriggerTypeCode_t::TRIGGER_DISTANCE) {
	this->setReferencePoint(defaultReferencePoint);
	this->setTriggerDistance(0.0);
}

Trigger::TriggerReturnCode_t DistanceTrigger::update(ObjectDataType newValue) {
	double networkDelayCorrection_m = 0.0, networkDelay_s = 0.0;
	struct timeval currentTime, triggerObjectNetworkDelay, actionObjectNetworkDelay;

	if (!newValue.MonrData.position.isPositionValid || !referencePoint.isPositionValid) {
		throw std::logic_error("Unable to update distance trigger on invalid data");
	}

	// Correct for two-way network delay effects on trigger distance
	if (newValue.MonrData.speed.isLongitudinalValid && newValue.MonrData.isTimestampValid) {
		TimeSetToCurrentSystemTime(&currentTime);
		timersub(&currentTime, &newValue.MonrData.timestamp, &triggerObjectNetworkDelay);

		// TODO Get rid of this false assumption
		actionObjectNetworkDelay = triggerObjectNetworkDelay;

		// Network delay consists of both positional data reporting delay and action execution message delay
		networkDelay_s = fabs(static_cast<double>(triggerObjectNetworkDelay.tv_sec)
							  + static_cast<double>(triggerObjectNetworkDelay.tv_usec) / 1000000.0)
						+ fabs(static_cast<double>(actionObjectNetworkDelay.tv_sec)
							   + static_cast<double>(actionObjectNetworkDelay.tv_usec) / 1000000.0);

		// Predict position offset
		networkDelayCorrection_m = networkDelay_s * newValue.MonrData.speed.longitudinal_m_s;
	}
	else {
		LogMessage(LOG_LEVEL_WARNING, "Invalid monitor data speed or timestamp: cannot correct for network delay");
	}

	switch (this->oper) {
	case LESS_THAN:
		return update(static_cast<bool>(UtilIsPositionNearTarget(newValue.MonrData.position, this->referencePoint, this->triggerDistance_m + networkDelayCorrection_m)),
				newValue.MonrData.timestamp);
	case GREATER_THAN:
		return update(static_cast<bool>(!UtilIsPositionNearTarget(newValue.MonrData.position, this->referencePoint, this->triggerDistance_m - networkDelayCorrection_m)),
				newValue.MonrData.timestamp);
	}
	throw std::logic_error("Distance trigger unimplemented operator");
}

Trigger::TriggerReturnCode_t DistanceTrigger::parseParameters() {
	Trigger::TriggerReturnCode_t retval = NOT_OK;
	if (parameters.size() == 1) {
		switch (parameters.front()) {
		case TRIGGER_PARAMETER_LESS_THAN:
		case TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO:
			// TODO check value to compare against
			this->oper = LESS_THAN;
			this->mode = HIGH;
			this->isStateTrue = false;
			this->wasStateTrue = false;
			retval = OK;
			break;
		case TRIGGER_PARAMETER_GREATER_THAN:
		case TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO:
			// TODO check value to compare against
			this->oper = GREATER_THAN;
			this->mode = HIGH;
			this->isStateTrue = false;
			this->wasStateTrue = false;
			retval = OK;
			break;
		default:
			return INVALID_ARGUMENT;
		}
	}
	else return INVALID_ARGUMENT;

	LogMessage(LOG_LEVEL_INFO, "Distance trigger configured with reference point (%.3f, %.3f, %.3f) and %.3f m trigger distance",
			   this->referencePoint.xCoord_m, this->referencePoint.yCoord_m, this->referencePoint.zCoord_m, this->triggerDistance_m);
	return retval;
}


Trigger::TriggerReturnCode_t DistanceTrigger::appendParameter(std::string inputStr) {
	try {
		// String represented a trigger parameter defined by ISO
		TriggerParameter_t param = asParameterCode(inputStr);
		return Trigger::appendParameter(param);
	} catch (std::invalid_argument e) {
		// String may have represented a number
		return parseNumericParameter(inputStr);
	}
}

Trigger::TriggerReturnCode_t DistanceTrigger::parseNumericParameter(std::string inputStr) {
	std::istringstream ss(inputStr);
	const std::string xString = "x:";
	const std::string yString = "y:";
	const std::string zString = "z:";
	double param = 0.0;
	size_t stringPos;
	if (inputStr.find("TO:(") != std::string::npos && inputStr.find(")") != std::string::npos) {
		if ((stringPos = inputStr.find(xString)) != std::string::npos) {
			ss.str(inputStr.substr(stringPos+xString.length()));
			if (!(ss >> this->referencePoint.xCoord_m)) {
				throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
			}
		}
		else
			throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
		if ((stringPos = inputStr.find(yString)) != std::string::npos) {
			ss.str(inputStr.substr(stringPos+yString.length()));
			if (!(ss >> this->referencePoint.yCoord_m)) {
				throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
			}
		}
		else
			throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
		if ((stringPos = inputStr.find(zString)) != std::string::npos) {
			ss.str(inputStr.substr(stringPos+zString.length()));
			if (!(ss >> this->referencePoint.zCoord_m)) {
				throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
			}
		}
		else
			throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as reference point");
		this->referencePoint.isPositionValid = true;
	}
	else if (ss >> param) {
		this->triggerDistance_m = param;
	}
	else {
		throw std::invalid_argument("Distance trigger unable to parse " + inputStr + " as numeric parameter");
	}
	return OK;
}

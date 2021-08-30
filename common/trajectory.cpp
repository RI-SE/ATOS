#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "regexpatterns.hpp"
#include "logging.h"
#include "trajectory.hpp"

const std::regex Trajectory::fileHeaderPattern("TRAJECTORY;(" + RegexPatterns::intPattern + ");("
		+ RegexPatterns::namePattern + ");" + RegexPatterns::versionPattern + ";("
		+ RegexPatterns::intPattern + ");");
const std::regex Trajectory::fileLinePattern("LINE;(" + RegexPatterns::floatPattern + ");("
		+ RegexPatterns::floatPattern + ");(" + RegexPatterns::floatPattern + ");("
		+ RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ");("
		+ RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ")?;("
		+ RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ")?;("
		+ RegexPatterns::floatPattern + ");(" + RegexPatterns::intPattern + ");ENDLINE;");
const std::regex Trajectory::fileFooterPattern("ENDTRAJECTORY;");

Trajectory::Trajectory(const Trajectory& other) {
	this->id = other.id;
	this->name = other.name;
	this->version = other.version;
	this->points = std::vector<TrajectoryPoint>(other.points);
}

void Trajectory::initializeFromFile(const std::string &fileName) {

	using namespace std;
	char trajDirPath[PATH_MAX];
	string errMsg;
	smatch match;
	ifstream file;
	bool isHeaderParsedSuccessfully = false;
	unsigned long nPoints = 0;

	UtilGetTrajDirectoryPath(trajDirPath, sizeof (trajDirPath));
	string trajFilePath(trajDirPath);
	trajFilePath += fileName;

	file.open(trajFilePath);
	if (!file.is_open()) {
		throw ifstream::failure("Unable to open file <" + trajFilePath + ">");
	}

	string line;
	errMsg = "Encountered unexpected end of file while reading file <" + trajFilePath + ">";
	for (unsigned long lineCount = 0; getline(file, line); lineCount++) {
		if (lineCount == 0) {
			if (regex_search(line, match, this->fileHeaderPattern)) {
				this->id = stoi(match[1]);
				this->name = match[2];
				this->version = 0;
				nPoints = stoul(match[6]);
				this->points.reserve(nPoints);
				isHeaderParsedSuccessfully = true;
			}
			else {
				errMsg = "The header of trajectory file <" + trajFilePath + "> is badly formatted";
				break;
			}
		}
		else if (lineCount > 0 && !isHeaderParsedSuccessfully) {
			errMsg = "Attempt to parse trajectory file <" + trajFilePath + "> before encountering header";
			break;
		}
		else if (lineCount > nPoints + 1) {
			errMsg = "Trajectory line count of file <" + trajFilePath
					+ "> does not match specified line count";
			break;
		}
		else if (lineCount == nPoints + 1) {
			if (regex_search(line, match, fileFooterPattern)) {
				file.close();
				LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", trajFilePath.c_str());
				return;
			}
			else {
				errMsg = "Final line of trajectory file <" + trajFilePath + "> badly formatted";
				break;
			}
		}
		else {
			if (regex_search(line, match, fileLinePattern)) {
				TrajectoryPoint point;
				point.setTime(stod(match[1]));
				point.setXCoord(stod(match[2]));
				point.setYCoord(stod(match[3]));
				if (match[4].matched)
					point.setZCoord(stod(match[4]));
				point.setHeading(stod(match[5]));
				if (match[6].matched)
					point.setLongitudinalVelocity(stod(match[6]));
				if (match[7].matched)
					point.setLateralVelocity(stod(match[7]));
				if (match[8].matched)
					point.setLongitudinalAcceleration(stod(match[8]));
				if (match[9].matched)
					point.setLateralAcceleration(stod(match[9]));
				point.setCurvature(stod(match[10]));
				point.setMode(static_cast<Trajectory::TrajectoryPoint::ModeType>(stoi(match[11])));
                points.push_back(point);
			}
			else {
				errMsg = "Line " + to_string(lineCount) + " of trajectory file <"
						+ trajFilePath + "> badly formatted";
				break;
			}
		}
	}
	file.close();
	LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", trajFilePath.c_str());
	throw invalid_argument(errMsg);
}


CartesianPosition Trajectory::TrajectoryPoint::getISOPosition() const {
	CartesianPosition retval;
	retval.xCoord_m = this->getXCoord();
	retval.yCoord_m = this->getYCoord();
	try {
		retval.zCoord_m = this->getZCoord();
	} catch (std::out_of_range e) {
		LogMessage(LOG_LEVEL_WARNING, "Casting trajectory point to cartesian position: optional z value assumed to be 0");
		retval.zCoord_m = 0.0;
	}
	retval.heading_rad = this->getHeading();
	retval.isHeadingValid = true;
	retval.isPositionValid = true;
	return retval;
}

SpeedType Trajectory::TrajectoryPoint::getISOVelocity() const {
	SpeedType retval;
	try {
	   retval.longitudinal_m_s = getLongitudinalVelocity();
	   retval.isLongitudinalValid = true;
	} catch (std::out_of_range) {
	   retval.longitudinal_m_s = 0.0;
	   retval.isLongitudinalValid = false;
	}
	try {
	   retval.lateral_m_s = getLateralVelocity();
	   retval.isLateralValid = true;
	} catch (std::out_of_range) {
	   retval.lateral_m_s = 0.0;
	   retval.isLateralValid = false;
	}
	return retval;
}

AccelerationType Trajectory::TrajectoryPoint::getISOAcceleration() const {
	AccelerationType retval;
	try {
	   retval.longitudinal_m_s2 = getLongitudinalAcceleration();
	   retval.isLongitudinalValid = true;
	} catch (std::out_of_range) {
	   retval.longitudinal_m_s2 = 0.0;
	   retval.isLongitudinalValid = false;
	}
	try {
	   retval.lateral_m_s2 = getLateralAcceleration();
	   retval.isLateralValid = true;
	} catch (std::out_of_range) {
	   retval.lateral_m_s2 = 0.0;
	   retval.isLateralValid = false;
	}
	return retval;
}

/*!
 * \brief Trajectory::TrajectoryPoint::relativeTo
 * \param other
 * \return
 */
Trajectory::TrajectoryPoint Trajectory::TrajectoryPoint::relativeTo(
		const TrajectoryPoint &other) const {

	using namespace Eigen;
	TrajectoryPoint relative;

	relative.setTime(this->getTime());
	relative.setHeading(this->getHeading() - other.getHeading());

	AngleAxis R3(-other.getHeading(), Vector3d::UnitZ());
	Rotation2Dd R(-relative.getHeading());
	relative.setPosition(R3*(zeroNaNs(this->getPosition())
						 - zeroNaNs(other.getPosition())));

	auto thisVel = zeroNaNs(this->getVelocity());
	auto otherVel = zeroNaNs(other.getVelocity());
	relative.setVelocity(R.inverse()*(R*thisVel - otherVel));

	auto thisAcc = zeroNaNs(this->getAcceleration());
	auto otherAcc = zeroNaNs(other.getAcceleration());
	relative.setAcceleration(R.inverse()*(R*thisAcc - otherAcc));

	// K(t) = ||r'(t) x r''(t)|| / ||r'(t)||³
	auto rPrim = R*this->getVelocity();
	auto rBis = R*this->getAcceleration();
	Eigen::Vector3d rPrim3(rPrim[0], rPrim[1], 0.0);
	Eigen::Vector3d rBis3(rBis[0], rBis[1], 0.0);
	if (rPrim3.norm() > 0.001) {
		relative.setCurvature(rPrim3.cross(rBis3).norm()
							  / std::pow(rPrim3.norm(), 3));
	}
	else {
		relative.setCurvature(0.0);
	}

	relative.setMode(this->getMode());
	return relative;
}

Trajectory Trajectory::relativeTo(
		const Trajectory &other) const {
	using namespace Eigen;
	// TODO check that ranges are sorted by time
	if (this->version != other.version) {
		throw std::invalid_argument("Attempted to calculate relative trajectory "
									"for two trajectories with differing versions");
	}

	Trajectory relative;
	relative.id = this->id;
	relative.name = this->name + "_rel_" + other.name;
	relative.version = this->version;
	for (auto trajPt = this->points.begin(); trajPt != this->points.end(); ++trajPt) {
		auto nearestTrajPtInOther = getNearest(other.points.begin(), other.points.end(), trajPt->getTime());
		// TODO maybe a check on time difference here
		relative.points.push_back(trajPt->relativeTo(*nearestTrajPtInOther));
	}

	return relative;
}

Trajectory::const_iterator Trajectory::getNearest(
		const_iterator first,
		const_iterator last,
		const double &time) {
	// Assumption: input range is sorted by time
	// Get first element with larger time than requested
	const_iterator after = std::lower_bound(first, last, time, [](const TrajectoryPoint& trajPt, const double& t) {
		return trajPt.getTime() < t;
	});

	if (after == first) return first;
	if (after == last)  return last - 1;

	const_iterator before = after - 1;

	// Return the element nearest to the requested time
	return (after->getTime() - time) < (time - before->getTime()) ? after : before;
}

std::string Trajectory::TrajectoryPoint::getFormatString() const {
	return "x:[m], y:[m], z:[m], hdg:[rad CW from N], "
		   "vx:[m/s,longitudinal], vy:[m/s,lateral], "
		   "ax:[m/s2,longitudinal], ay:[m/s2,lateral], "
		   "c:[1/m], md:[]";
}

std::string Trajectory::TrajectoryPoint::toString() const {
	std::string retval = "";
	std::stringstream ss(retval);
	ss << "x:" << position[0] << ", "
	   << "y:" << position[1] << ", "
	   << "z:" << position[2] << ", "
	   << "hdg:" << heading << ", "
	   << "vx:" << velocity[0] << ", "
	   << "vy:" << velocity[1] << ", "
	   << "ax:" << acceleration[0] << ", "
	   << "ay:" << acceleration[1] << ", "
	   << "c:" << curvature << ", "
	   << "md:" << mode;
	return ss.str();
}

std::string Trajectory::toString() const {
	std::string retval = "";
	std::stringstream ss(retval);
	ss << "Trajectory:"
	   << "\n Name: " << this->name
	   << "\n ID: " << this->id
	   << "\n Version: " << this->version;
	for (const auto& point : points) {
		ss << "\n\t" << point.toString();
	}
	return ss.str();
}

void Trajectory::constrainVelocityTo(double vel_m_s){

	auto point = points.rbegin();
		while (point != points.rend()) {
				if(point->getLongitudinalVelocity() > vel_m_s){
					point->setLongitudinalAcceleration(0);
					point->setLongitudinalVelocity(vel_m_s);
			}
			point++;
		}

		//TODO: something to take lateral into account.

	this->name = this->name + "_" + std::to_string(vel_m_s) + "MS";
}


void Trajectory::addAccelerationTo(double vel_m_s){

	auto point = points.begin();
	int a = 5;
	int b = -2;
		while (point != points.end()) {
						
			point->setLongitudinalVelocity((vel_m_s / (1 + (exp(a + (b * point->getTime()))))));
			point->setLongitudinalAcceleration(-((b*point->getTime()*exp(((b*point->getTime()))+a)) /(pow(1 + (exp(a + (b * point->getTime()))),2))));

			if(vel_m_s / (1 + (exp(a + (b * point->getTime()))))==vel_m_s){
				break;
			}
			point++;
        }
}

void Trajectory::addWilliamsonTurn(double turnRadius = 5, TrajectoryPoint startPoint = TrajectoryPoint(), double startTime = 0)
{

    std::cout << std::endl << "X: " << startPoint.getXCoord() << std::endl;
    std::cout << "Y: " << startPoint.getYCoord() << std::endl ;
    std::cout << "H: " << startPoint.getHeading() << std::endl ;

    using Eigen::MatrixXd;

    double acceleration = 1;        //m/s/s
    double topSpeed = 2.7777;
    const int noOfPoints = 500;
    double radius = turnRadius;
    double heading = startPoint.getHeading();
    double headingRad = heading;//(heading-90) * M_PI/180;

    Eigen::Matrix<double, 2,2> rotM;
    Eigen::VectorXd theta0;         //First section
    Eigen::VectorXd theta1;         //Second section
    Eigen::VectorXd endStraight;    //Third section
    Eigen::Matrix<double, 2, noOfPoints> xyM;
    Eigen::Matrix<double, 2, noOfPoints> resM;
    Eigen::Array<double, 1,noOfPoints> headingArray;
    Eigen::Array<double, 1,noOfPoints> speedArray;
    Eigen::Array<double, 1,noOfPoints> accelerationArray;
    Eigen::Array<double, 1,noOfPoints> timeArray;


    //Calculate length of each section
    double len0 = (M_PI * 2 * radius) / 4;
    double len1 = 3 * ((M_PI * 2 * radius) / 4);
    double len2 = radius * 2;
    double totalLength = len0 + len1 + len2;

    //First section
    int n0 = int(noOfPoints * (len0 / totalLength));
    theta0 = Eigen::VectorXd::LinSpaced(n0, M_PI, M_PI_2);

    for(int i = 0; i < theta0.size(); i++)
    {
        xyM(1,i) = radius * cos(theta0[i]) + radius;
        xyM(0,i) = radius * sin(theta0[i]);
        headingArray[i] = theta0[i] - M_PI_2;
    }

    //second section
    int n1 = int(noOfPoints * (len1 / totalLength));
    theta1 = Eigen::VectorXd::LinSpaced(n1, -1*M_PI_2, M_PI);

    for(int i = 0; i < theta1.size(); i++)
    {
        xyM(1,i+n0) = radius * cos(theta1[i]) + radius;
        xyM(0,i+n0) = radius * sin(theta1[i]) + 2 * radius;
        headingArray[i+n0] = theta1[i] + M_PI_2;
    }

    //third section
    int n2 = int(noOfPoints * (len2 / totalLength));
    endStraight = Eigen::VectorXd::LinSpaced(n2, radius * 2, 0);
    for(int i = 0; i < n2; i++)
    {
        xyM(1,i+n0+n1) = 0;
        xyM(0,i+n0+n1) = endStraight[i];
        headingArray[i+n0+n1] = -1*M_PI_2;
    }

    //Values for rotation matrix
    rotM <<   cos(headingRad), -sin(headingRad),
                sin(headingRad), cos(headingRad);

    resM = rotM * xyM;

    int actualNoOfPoints = n0+n1+n2;

    //Offset result matrix
    for(int i = 0; i < actualNoOfPoints; i++)
    {
        resM(0,i) += startPoint.getXCoord();
        resM(1,i) += startPoint.getYCoord();
    }

    Eigen::ArrayXd oneArray = Eigen::ArrayXd::Ones(noOfPoints);
    headingArray += headingRad*oneArray;


    //Speed

    //AccelerationSection
    double accelerationPeriod = topSpeed / acceleration;                //t = (v - u) / a
    double accelerationDistance = ((pow(topSpeed, 2))/acceleration)/2;

    //Topspeed section
    double topSpeedDistance = totalLength - accelerationDistance*2;
    double topSpeedPeriod = topSpeedDistance / topSpeed;

    double totalRuntime = accelerationPeriod + topSpeedPeriod + accelerationPeriod; //Accelerate -> Top Speed -> Decelerate

    double timeStep = totalRuntime / noOfPoints;


    //Speed for each point
    double currSpeed = 0;
    for(int i = 0; i < noOfPoints; i++)
    {
        timeArray[i] = i*timeStep;

        if(timeArray[i] < accelerationPeriod)
        {
            currSpeed = i * (acceleration*timeStep);
            accelerationArray[i] = -acceleration;
        }
        else if(timeArray[i] < topSpeedPeriod + accelerationPeriod)
        {
            if(currSpeed > topSpeed)
            {
                currSpeed = topSpeed;
            }
            accelerationArray[i] = 0;
        }
        else
        {
            currSpeed -= acceleration*timeStep;
            accelerationArray[i] = acceleration;
        }
        speedArray[i] = currSpeed;

    }

    //create trajectory points
    std::vector<TrajectoryPoint> tempVector;
    for(int i = 0; i < actualNoOfPoints; i++)
    {

        TrajectoryPoint tempPoint;
        tempPoint.setTime(timeArray[i]+startTime);
        tempPoint.setXCoord(resM(0,i));
        tempPoint.setYCoord(resM(1,i));
        tempPoint.setZCoord(0.00000);
        tempPoint.setHeading(-headingArray[i] - M_PI_2);
        tempPoint.setLongitudinalVelocity(speedArray[i]);
        tempPoint.setLateralVelocity(0.00000);
        tempPoint.setLongitudinalAcceleration(accelerationArray[i]);
        tempPoint.setLateralAcceleration(0.00000);
        tempPoint.setCurvature(0);

        tempVector.push_back(tempPoint);
    }

    points.insert(std::end(points), std::begin(tempVector), std::end(tempVector));


}

void Trajectory::reverse(double startTime = 0){
	if(points.empty()){
		throw std::invalid_argument("Attempted to reverse non existing trajectory");
	}

	this->name = this->name + "_Reversed";

	std::reverse(points.begin(), points.end());

	std::vector<double> timeVector;

	auto point = points.rbegin();
		while (point != points.rend()) {
			point->setHeading(point->getHeading()-M_PI);
			point->setCurvature(point->getCurvature()*-1);
			try{
				point->setLateralVelocity(point->getLateralVelocity()*-1);
			}
			catch(std::out_of_range){
				std::cout<<"Ignoring uninitialized lateral velocity";
			}
			try{
				point->setLateralAcceleration(point->getLateralAcceleration()*-1);
			}
			catch(std::out_of_range){
				std::cout<<"Ignoring uninitialized lateral acceleration";
			}
            timeVector.push_back(point->getTime()+startTime);
			point++;
		}

	point = points.rbegin();
		while (point != points.rend()) {
			point->setTime(timeVector.back());
			timeVector.pop_back();
			point++;
		}
}


void Trajectory::saveToFile(const std::string& fileName) {
	using std::string, std::smatch, std::ofstream;
	char trajDirPath[PATH_MAX];

	UtilGetTrajDirectoryPath(trajDirPath, sizeof (trajDirPath));
	string trajFilePath(trajDirPath);
	trajFilePath += fileName;

	ofstream outputTraj;
	LogMessage(LOG_LEVEL_DEBUG, "Opening file %s", trajFilePath.c_str());
	outputTraj.open (trajFilePath);
	LogMessage(LOG_LEVEL_DEBUG, "Outputting trajectory to file");
	outputTraj << "TRAJECTORY;" << this->id <<";" << this->name << ";" << this->version << ";" << this->points.size() << ";" <<  "\n";
	for (const auto& point : points) {
		outputTraj << "LINE;"
		<< std::fixed << std::setprecision(2) << point.getTime() << ";"
		<< std::fixed << std::setprecision(6) << point.getXCoord() << ";"
		<< std::fixed << std::setprecision(6) << point.getYCoord() << ";"
		<< std::fixed << std::setprecision(6) << point.getZCoord() << ";"
		<< std::fixed << std::setprecision(6) << point.getHeading() << ";"
		<< std::fixed << std::setprecision(6) << point.getLongitudinalVelocity() << ";"
		<< ";" //point.getLateralVelocity() << ";"
		<<  std::fixed << std::setprecision(6) <<(point.getLongitudinalAcceleration()) << ";"
		<< ";" //point.getLateralAcceleration() << ";"
		<< std::fixed << std::setprecision(6) << point.getCurvature() << ";"
		<< std::fixed << std::setprecision(6) << point.getMode()
		<<";ENDLINE;" << "\n";
	}
	outputTraj << "ENDTRAJECTORY;" <<  "\n";

	outputTraj.close();
	LogMessage(LOG_LEVEL_DEBUG, "Closed file %s", trajFilePath.c_str());
}

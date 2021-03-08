#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <linux/limits.h>
#include <vector>
#include <iostream>
#include <exception>
#include <regex>
#include <functional>
#include <Eigen/Dense>

#include "util.h"

class Trajectory {
public:
    class TrajectoryPoint {
    public:
        typedef enum {
            CONTROLLED_BY_DRIVE_FILE,
            CONTROLLED_BY_VEHICLE
        } ModeType;

		TrajectoryPoint() {
			position[2] = std::numeric_limits<double>::quiet_NaN();
			velocity[0] = std::numeric_limits<double>::quiet_NaN();
			velocity[1] = std::numeric_limits<double>::quiet_NaN();
			acceleration[0] = std::numeric_limits<double>::quiet_NaN();
			acceleration[1] = std::numeric_limits<double>::quiet_NaN();
		}

		~TrajectoryPoint() { }

		TrajectoryPoint relativeTo(const TrajectoryPoint& other) const;

		void setTime(const double& value) { time = value; }
		void setPosition(const Eigen::Vector3d& value) { position = value; }
		void setXCoord(const double& value) { position[0] = value; }
		void setYCoord(const double& value) { position[1] = value; }
		void setZCoord(const double& value) { position[2] = value; }
		void setHeading(const double& value) { heading = value; }
		void setVelocity(const Eigen::Vector2d& value) { velocity = value; }
		void setLongitudinalVelocity(const double& value) { velocity[0] = value; }
		void setLateralVelocity(const double& value) { velocity[1] = value; }
		void setAcceleration(const Eigen::Vector2d& value) { acceleration = value; }
		void setLongitudinalAcceleration(const double& value) { acceleration[0] = value; }
		void setLateralAcceleration(const double& value) { acceleration[1] = value; }
		void setCurvature(const double& value) { curvature = value; }
		void setMode(const ModeType& value) { mode = value; }

		double getTime() const { return time; }
		Eigen::Vector3d getPosition() const { return position; }
		double getXCoord() const { return position[0]; }
		double getYCoord() const { return position[1]; }
        double getZCoord() const {
			if (std::isnan(position[2]))
                throw std::out_of_range("Uninitialized member z");
            else
				return position[2];
        }
		double getHeading() const { return heading; }
		CartesianPosition getCartesianPosition() const;
		Eigen::Vector2d getVelocity() const { return velocity; }
        double getLongitudinalVelocity() const {
			if (std::isnan(velocity[0]))
                throw std::out_of_range("Uninitialized member longitudinal velocity");
            else
				return velocity[0];
        }
        double getLateralVelocity() const {
			if (std::isnan(velocity[1]))
                throw std::out_of_range("Uninitialized member lateral velocity");
            else
				return velocity[1];
        }
		Eigen::Vector2d getAcceleration() const { return acceleration; }
        double getLongitudinalAcceleration() const {
			if (std::isnan(acceleration[0]))
                throw std::out_of_range("Uninitialized member longitudinal acceleration");
            else
				return acceleration[0];
        }
        double getLateralAcceleration() const {
			if (std::isnan(acceleration[1]))
                throw std::out_of_range("Uninitialized member lateral acceleration");
            else
				return acceleration[1];
        }
		double getCurvature() const { return this->curvature; }
        ModeType getMode() const { return this->mode; }

    private:
        double time = 0;
		Eigen::Vector3d position; //! x, y, z [m]
		double heading = 0;		  //! heading from north? [rad]
		Eigen::Vector2d velocity; //! Vehicle frame, x forward [m/s]
		Eigen::Vector2d acceleration; //! Vehicle frame, x forward [m/s]
        double curvature = 0;
        ModeType mode = CONTROLLED_BY_VEHICLE;
    };
	typedef std::vector<TrajectoryPoint>::iterator iterator;
	typedef std::vector<TrajectoryPoint>::const_iterator const_iterator;

    Trajectory() = default;
    ~Trajectory() { points.clear(); }
    Trajectory(const Trajectory& other);
    std::vector<TrajectoryPoint> points;
    std::string name = "";
    int version = 0;
	int id = 0;

	void initializeFromFile(const std::string& fileName);
	Trajectory relativeTo(const Trajectory& other) const;
	static const_iterator getNearest(const_iterator first, const_iterator last, const double& time);

	std::vector<double> getTimes() const { return collect(std::mem_fn(&TrajectoryPoint::getTime)); }
	std::vector<Eigen::Vector3d> getPositions() const { return collect(std::mem_fn(&TrajectoryPoint::getPosition)); }
	std::vector<double> getXCoords() const { return collect(std::mem_fn(&TrajectoryPoint::getXCoord)); }
	std::vector<double> getYCoords() const { return collect(std::mem_fn(&TrajectoryPoint::getYCoord)); }
	std::vector<double> getZCoords() const { return collect(std::mem_fn(&TrajectoryPoint::getZCoord)); }
	std::vector<double> getHeadings() const { return collect(std::mem_fn(&TrajectoryPoint::getHeading)); }
	std::vector<Eigen::Vector2d> getVelocities() const { return collect(std::mem_fn(&TrajectoryPoint::getVelocity)); }
	std::vector<double> getLongitudinalVelocities() const { return collect(std::mem_fn(&TrajectoryPoint::getLongitudinalVelocity)); }
	std::vector<double> getLateralVelocities() const { return collect(std::mem_fn(&TrajectoryPoint::getLateralVelocity)); }
	std::vector<Eigen::Vector2d> getAccelerations() const { return collect(std::mem_fn(&TrajectoryPoint::getAcceleration)); }
	std::vector<double> getLongitudinalAccelerations() const { return collect(std::mem_fn(&TrajectoryPoint::getLongitudinalAcceleration)); }
	std::vector<double> getLateralAccelerations() const { return collect(std::mem_fn(&TrajectoryPoint::getLateralAcceleration)); }
	std::vector<double> getCurvatures() const { return collect(std::mem_fn(&TrajectoryPoint::getCurvature)); }
	std::vector<TrajectoryPoint::ModeType> getModes() const { return collect(std::mem_fn(&TrajectoryPoint::getMode)); }

	void setTimes(const std::vector<double>& times) { assign(std::mem_fn(&TrajectoryPoint::setTime), times); }
	void setXCoords(const std::vector<double>& xCoords) { assign(std::mem_fn(&TrajectoryPoint::setXCoord), xCoords); }
	void setYCoords(const std::vector<double>& yCoords) { assign(std::mem_fn(&TrajectoryPoint::setYCoord), yCoords); }
	void setZCoords(const std::vector<double>& zCoords) { assign(std::mem_fn(&TrajectoryPoint::setZCoord), zCoords); }
	void setHeadings(const std::vector<double>& headings) { assign(std::mem_fn(&TrajectoryPoint::setHeading), headings); }
	void setLongitudinalVelocities(const std::vector<double>& velocities) {
		assign(std::mem_fn(&TrajectoryPoint::setLongitudinalVelocity), velocities); }
	void setLateralVelocities(const std::vector<double>& velocities) {
		assign(std::mem_fn(&TrajectoryPoint::setLateralVelocity), velocities); }
	void setLongitudinalAccelerations(const std::vector<double>& accelerations) {
		assign(std::mem_fn(&TrajectoryPoint::setLongitudinalAcceleration), accelerations); }
	void setLateralAccelerations(const std::vector<double>& accelerations) {
		assign(std::mem_fn(&TrajectoryPoint::setLateralAcceleration), accelerations); }
	void setCurvatures(const std::vector<double>& curvatures) {
		assign(std::mem_fn(&TrajectoryPoint::setCurvature), curvatures); }
	void setModes(const std::vector<TrajectoryPoint::ModeType>& modes) {
		assign(std::mem_fn(&TrajectoryPoint::setMode), modes); }
private:
	static const std::regex fileHeaderPattern;
	static const std::regex fileLinePattern;
	static const std::regex fileFooterPattern;

	//! Creates a STL vector based on calls to the specified getter
	template<typename T>
	std::vector<T> collect(std::_Mem_fn<T (Trajectory::TrajectoryPoint::*)() const> f) const {
		std::vector<T> output;
		std::transform(points.begin(), points.end(), std::back_inserter(output), f);
		return output;
	}

	template<typename T>
	void assign(std::_Mem_fn<void (Trajectory::TrajectoryPoint::*)(const T&)> f, const std::vector<T>& input) {
		if (input.size() != points.size()) {
			throw std::out_of_range("Attempted to set trajectory point elements to vector of differing size");
		}
		std::transform(input.begin(), input.end(), points.begin(), f);
	}
};


#endif

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

		std::string toString() const;
		std::string getFormatString() const;
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

	std::string toString() const;

private:
	static const std::regex fileHeaderPattern;
	static const std::regex fileLinePattern;
	static const std::regex fileFooterPattern;
};


#endif

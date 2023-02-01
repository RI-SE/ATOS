#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <limits>
#include <vector>
#include <iostream>
#include <exception>
#include <regex>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include "atos_interfaces/msg/cartesian_trajectory.hpp"

#include "util.h"
namespace ATOS {
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

		template<class Rep,class Period>
		void setTime(const std::chrono::duration<Rep,Period>& value) { time = value; }
		void setTime(const double value) { const auto timeVar = static_cast<unsigned long>( value * 1000 ); time = std::chrono::milliseconds(timeVar); }
		void setPosition(const Eigen::Vector3d& value) { position = value; }
		void setXCoord(const double& value) { position[0] = value; }
		void setYCoord(const double& value) { position[1] = value; }
		void setZCoord(const double& value) { position[2] = value; }
		void setHeading(const double& value) { heading = std::fmod(value, 2*M_PI);
											   heading += heading < 0 ? 2*M_PI : 0.0; }
		void setVelocity(const Eigen::Vector2d& value) { velocity = value; }
		void setLongitudinalVelocity(const double& value) { velocity[0] = value; }
		void setLateralVelocity(const double& value) { velocity[1] = value; }
		void setAcceleration(const Eigen::Vector2d& value) { acceleration = value; }
		void setLongitudinalAcceleration(const double& value) { acceleration[0] = value; }
		void setLateralAcceleration(const double& value) { acceleration[1] = value; }
		void setCurvature(const double& value) { curvature = value; }
		void setMode(const ModeType& value) { mode = value; }

		std::chrono::milliseconds getTime() const { return time; }
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

		CartesianPosition getISOPosition() const;
		SpeedType getISOVelocity() const;
		AccelerationType getISOAcceleration() const;

		std::string toString() const;
		std::string getFormatString() const;
	private:
		std::chrono::milliseconds time = std::chrono::milliseconds(0);
		Eigen::Vector3d position; //! x, y, z [m]
		double heading = 0;		  //! Heading ccw from x axis [rad]
		Eigen::Vector2d velocity; //! Vehicle frame, x forward [m/s]
		Eigen::Vector2d acceleration; //! Vehicle frame, x forward [m/s]
		double curvature = 0;
		ModeType mode = CONTROLLED_BY_VEHICLE;

		template<typename T, int rows>
		Eigen::Matrix<T, rows, 1> zeroNaNs(
				const Eigen::Matrix<T, rows, 1>& v) const {
			static_assert(v.SizeAtCompileTime != 0, "Zero size matrix passed to zeroNaNs");
			Eigen::Matrix<T, rows, 1> ret(v);
			for (int i = 0; i < ret.SizeAtCompileTime; ++i) {
				ret[i] = isnan(ret[i]) ? 0.0 : ret[i];
			}
			return ret;
		}
	};
	typedef std::vector<TrajectoryPoint>::iterator iterator;
	typedef std::vector<TrajectoryPoint>::const_iterator const_iterator;
	typedef std::vector<TrajectoryPoint>::reverse_iterator reverse_iterator;
	typedef std::vector<TrajectoryPoint>::const_reverse_iterator const_reverse_iterator;

	Trajectory() = default;
	~Trajectory() { points.clear(); }
	Trajectory(const Trajectory& other);
	std::vector<TrajectoryPoint> points;
	std::string name = "";
	unsigned short version = 0;
	unsigned short id = 0;

	void initializeFromFile(const std::string& fileName);
	void initializeFromCartesianTrajectory(const atos_interfaces::msg::CartesianTrajectory& cartesianTrajectory);
	Trajectory relativeTo(const Trajectory& other) const;
	static const_iterator getNearest(const_iterator first, const_iterator last, const double& time);
	std::string toString() const;
	atos_interfaces::msg::CartesianTrajectory toCartesianTrajectory();
	std::size_t size() const { return points.size(); }

	void saveToFile(const std::string& fileName) const;
	Trajectory reversed() const;
	Trajectory rescaledToVelocity(const double vel_m_s) const;
	static Trajectory createWilliamsonTurn(double turnRadius = 5, double acceleration = 1, TrajectoryPoint startPoint = TrajectoryPoint(), std::chrono::milliseconds startTime = std::chrono::milliseconds(0));

	Trajectory appendedWith(const Trajectory& other);
	template<class Rep,class Period>
	Trajectory delayed(const std::chrono::duration<Rep,Period>& delay) const {
		Trajectory newTrajectory = Trajectory(*this);
		newTrajectory.name = newTrajectory.name + "_delayed";
		for (auto& trajPt : newTrajectory.points) {
			trajPt.setTime(trajPt.getTime() + delay);
		}
		return newTrajectory;
	}

	bool isValid() const;
private:
	static const std::regex fileHeaderPattern;
	static const std::regex fileLinePattern;
	static const std::regex fileFooterPattern;

	bool areTimestampsIncreasing() const;
};
} // namespace ATOS

#endif

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <linux/limits.h>
#include <vector>
#include <iostream>
#include <exception>
#include <regex>

#include "util.h"

class Trajectory {
public:
    class TrajectoryPoint {
    public:
        typedef enum {
            CONTROLLED_BY_DRIVE_FILE,
            CONTROLLED_BY_VEHICLE
        } ModeType;

        TrajectoryPoint() = default;

        ~TrajectoryPoint() {
            delete zCoord;
            delete longitudinalVelocity;
            delete lateralVelocity;
            delete longitudinalAcceleration;
            delete lateralAcceleration;
            zCoord = nullptr;
            longitudinalVelocity = nullptr;
            lateralVelocity = nullptr;
            longitudinalAcceleration = nullptr;
            lateralAcceleration = nullptr;
        }

        TrajectoryPoint(const TrajectoryPoint& other) {
            this->setTime(other.getTime());
            this->setXCoord(other.getXCoord());
            this->setYCoord(other.getYCoord());
            try {
                this->setZCoord(other.getZCoord());
            } catch (std::out_of_range) {
                this->zCoord = nullptr;
            }
            this->setHeading(other.getHeading());
            try {
                this->setLongitudinalVelocity(other.getLongitudinalVelocity());
            } catch (std::out_of_range) {
                this->longitudinalVelocity = nullptr;
            }
            try {
                this->setLateralVelocity(other.getLateralVelocity());
            } catch (std::out_of_range) {
                this->lateralVelocity = nullptr;
            }
            try {
                this->setLongitudinalAcceleration(other.getLongitudinalAcceleration());
            } catch (std::out_of_range) {
                this->longitudinalAcceleration = nullptr;
            }
            try {
                this->setLateralAcceleration(other.getLateralAcceleration());
            } catch (std::out_of_range) {
                this->lateralAcceleration = nullptr;
            }
            this->setCurvature(other.getCurvature());
            this->setMode(other.getMode());
        }


        void setTime(double value) { this->time = value; }
        void setXCoord(double value) { this->xCoord = value; }
        void setYCoord(double value) { this->yCoord = value; }
        void setZCoord(double value) {
            if (this->zCoord == nullptr)
                this->zCoord = new double(value);
            else
                *this->zCoord = value;
        }
        void setHeading(double value) {this->heading = value; }
        void setLongitudinalVelocity(double value) {
            if (this->longitudinalVelocity == nullptr)
                this->longitudinalVelocity = new double(value);
            else
                *this->longitudinalVelocity = value;
        }
        void setLateralVelocity(double value) {
            if (this->lateralVelocity == nullptr)
                this->lateralVelocity = new double(value);
            else
                *this->lateralVelocity = value;
        }
        void setLongitudinalAcceleration(double value) {
            if (this->longitudinalAcceleration == nullptr)
                this->longitudinalAcceleration = new double(value);
            else
                *this->longitudinalAcceleration = value;
        }
        void setLateralAcceleration(double value) {
            if (this->lateralAcceleration == nullptr)
                this->lateralAcceleration = new double(value);
            else
                *this->lateralAcceleration = value;
        }
        void setCurvature(double value) { this->curvature = value; }
        void setMode(ModeType value) { this->mode = value; }

        double getTime() const { return this->time; }
        double getXCoord() const { return this->xCoord; }
        double getYCoord() const { return this->yCoord; }
        double getZCoord() const {
            if (this->zCoord == nullptr)
                throw std::out_of_range("Uninitialized member z");
            else
                return *this->zCoord;
        }
        double getHeading() const { return this->heading; }
		CartesianPosition getCartesianPosition() const;
        double getLongitudinalVelocity() const {
            if (this->longitudinalVelocity == nullptr)
                throw std::out_of_range("Uninitialized member longitudinal velocity");
            else
                return *this->longitudinalVelocity;
        }
        double getLateralVelocity() const {
            if (this->lateralVelocity == nullptr)
                throw std::out_of_range("Uninitialized member lateral velocity");
            else
                return *this->lateralVelocity;
        }
        double getLongitudinalAcceleration() const {
            if (this->longitudinalAcceleration == nullptr)
                throw std::out_of_range("Uninitialized member longitudinal acceleration");
            else
                return *this->longitudinalAcceleration;
        }
        double getLateralAcceleration() const {
            if (this->lateralAcceleration == nullptr)
                throw std::out_of_range("Uninitialized member lateral acceleration");
            else
                return *this->lateralAcceleration;
        }
        double getCurvature() const { return this->yCoord; }
        ModeType getMode() const { return this->mode; }

    private:
        double time = 0;
        double xCoord = 0;
        double yCoord = 0;
        double *zCoord = nullptr;
        double heading = 0;
        double *longitudinalVelocity = nullptr;
        double *lateralVelocity = nullptr;
        double *longitudinalAcceleration = nullptr;
        double *lateralAcceleration = nullptr;
        double curvature = 0;
        ModeType mode = CONTROLLED_BY_VEHICLE;
    };

    Trajectory() = default;
    ~Trajectory() { points.clear(); }
    Trajectory(const Trajectory& other);
    std::vector<TrajectoryPoint> points;
    std::string name = "";
    int version = 0;
	int id = 0;

    void initializeFromFile(const std::string fileName);
private:
    static const std::regex fileHeaderPattern;
    static const std::regex fileLinePattern;
    static const std::regex fileFooterPattern;
};
#endif

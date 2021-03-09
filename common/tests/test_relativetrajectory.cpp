#include "../trajectory.hpp"
#include <exception>
#include <cmath>
#include <vector>
#include <iostream>
#define TIME_TOL_S 0.000001
#define POS_TOL_M 0.001
#define HDG_TOL_DEG 0.1

using traj_pt = Trajectory::TrajectoryPoint;
static void time_test();
static void position_test();
static void heading_test();
static void velocity_test();
static void acceleration_test();
static void curvature_test();
static void mode_test();
static void set_default(traj_pt&);

int main(int argc, char** argv) {
	try {
		time_test();
		position_test();
		heading_test();
		velocity_test();
		acceleration_test();
		curvature_test();
		mode_test();
		exit(EXIT_SUCCESS);
	}
	catch (std::runtime_error& e) {
		std::cerr << "Test " << __FILE__ << " failed: " << std::endl
				  << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
}

void set_default(traj_pt& pt) {
	pt.setTime(0.0);
	pt.setXCoord(0.0);
	pt.setYCoord(0.0);
	pt.setZCoord(0.0);
	pt.setHeading(0.0);
	pt.setLongitudinalVelocity(0.0);
	pt.setLateralVelocity(0.0);
	pt.setLongitudinalAcceleration(0.0);
	pt.setLateralAcceleration(0.0);
	pt.setCurvature(0.0);
	pt.setMode(Trajectory::TrajectoryPoint::CONTROLLED_BY_DRIVE_FILE);
}

void time_test() {
	traj_pt p1, p2;
	set_default(p1);
	set_default(p2);

	std::vector<double> t1s =
	{0.0, 0.1, 0.1, 0.2};
	std::vector<double> t2s =
	{0.0, 0.1, 0.2, 0.1};
	std::vector<double> out = t1s;

	for (unsigned long i=0; i < out.size(); ++i) {
		p1.setTime(t1s[i]);
		p2.setTime(t2s[i]);
		auto pRel = p1.relativeTo(p2);
		if (std::abs(pRel.getTime() - out[i]) > TIME_TOL_S) {
			std::stringstream ss;
			ss << "Timestamp after getting relative not within error tolerance:" << std::endl;
			ss << "p1:  " << t1s[i] << std::endl;
			ss << "p2:  " << t2s[i] << std::endl;
			ss << "exp: " << out[i] << std::endl;
			ss << "act: " << pRel.getTime();
			throw std::runtime_error(ss.str());
		}
	}
}

void position_test() {
	traj_pt p1, p2;
	set_default(p1);
	set_default(p2);
	typedef struct {
		double x;
		double y;
		double z;
	} pos;

	std::vector<pos> p1s =
	{{    0.0,    0.0,    0.0},
	 {   10.0,   -5.0,    2.2},
	 {  -10.0,    5.0,    2.2},
	 { 5001.0, 4001.0, 3001.0}};
	std::vector<pos> p2s =
	{{    0.0,    0.0,    0.0},
	 {    5.0,    5.0,    1.3},
	 {   10.0,   -5.0,    4.2},
	 {    1.0,    1.0,    1.0}};
	std::vector<pos> out =
	{{    0.0,    0.0,    0.0},
	 {    5.0,  -10.0,    0.9},
	 {  -20.0,   10.0,   -2.0},
	 { 5000.0, 4000.0, 3000.0}};

	for (unsigned long i=0; i < out.size(); ++i) {
		p1.setXCoord(p1s[i].x);
		p1.setYCoord(p1s[i].y);
		p1.setZCoord(p1s[i].z);
		p2.setXCoord(p2s[i].x);
		p2.setYCoord(p2s[i].y);
		p2.setZCoord(p2s[i].z);
		auto pRel = p1.relativeTo(p2);
		if (std::abs(pRel.getXCoord() - out[i].x) > POS_TOL_M
				|| std::abs(pRel.getYCoord() - out[i].y) > POS_TOL_M
				|| std::abs(pRel.getZCoord() - out[i].z) > POS_TOL_M) {
			std::stringstream ss;
			ss << "Relative position not within error tolerance:" << std::endl;
			ss << "p1:  (" << p1s[i].x << "," << p1s[i].y << "," << p1s[i].z << ")" << std::endl;
			ss << "p2:  (" << p2s[i].x << "," << p2s[i].y << "," << p2s[i].z << ")" << std::endl;
			ss << "exp: (" << out[i].x << "," << out[i].y << "," << out[i].z << ")" << std::endl;
			ss << "act: (" << pRel.getXCoord() << "," << pRel.getYCoord()
			   << "," << pRel.getZCoord() << ")";
			throw std::runtime_error(ss.str());
		}
	}
}

void heading_test() {
	traj_pt p1, p2;
	set_default(p1);
	set_default(p2);

	std::vector<double> h1s =
	{0.0, 90.0, -90.0, 0.0, 0.0,
	 20.0, 20.0, 20.0, -20.0, -20.0,
	 370.0, -180.0};
	std::vector<double> h2s =
	{0.0, 0.0, 0.0, 90.0, -90.0,
	 0.0, 20.0, 45.0, -50.0, -10.0,
	 0.0, 180.0};
	std::vector<double> out =
	{0.0, 90.0, -90.0, -90.0, 90.0,
	 20.0, 0.0, -25.0, 30.0, -10.0,
	 10.0, 0.0};

	for (unsigned long i=0; i < out.size(); ++i) {
		p1.setHeading(h1s[i]*M_PI/180.0);
		p2.setHeading(h2s[i]*M_PI/180.0);
		auto pRel = p1.relativeTo(p2);
		if (std::abs(pRel.getHeading()*180.0/M_PI - out[i]) > HDG_TOL_DEG) {
			std::stringstream ss;
			ss << "Relative heading not within error tolerance:" << std::endl;
			ss << "p1:  " << h1s[i] << std::endl;
			ss << "p2:  " << h2s[i] << std::endl;
			ss << "exp: " << out[i] << std::endl;
			ss << "act: " << pRel.getHeading()*180.0/M_PI;
			throw std::runtime_error(ss.str());
		}
	}
}

void velocity_test() {

}

void acceleration_test() {

}

void curvature_test() {

}

void mode_test() {
	traj_pt p1, p2;
	set_default(p1);
	set_default(p2);

	std::vector<traj_pt::ModeType> m1s =
	{traj_pt::CONTROLLED_BY_VEHICLE, traj_pt::CONTROLLED_BY_DRIVE_FILE,
	 traj_pt::CONTROLLED_BY_VEHICLE, traj_pt::CONTROLLED_BY_DRIVE_FILE};
	std::vector<traj_pt::ModeType> m2s =
	{traj_pt::CONTROLLED_BY_VEHICLE, traj_pt::CONTROLLED_BY_DRIVE_FILE,
	 traj_pt::CONTROLLED_BY_DRIVE_FILE, traj_pt::CONTROLLED_BY_VEHICLE};
	std::vector<traj_pt::ModeType> out = m1s;

	for (unsigned long i=0; i < out.size(); ++i) {
		p1.setMode(m1s[i]);
		p2.setMode(m2s[i]);
		auto pRel = p1.relativeTo(p2);
		if (pRel.getMode() != out[i]) {
			std::stringstream ss;
			ss << "Mode after getting relative not correct:" << std::endl;
			ss << "p1:  " << m1s[i] << std::endl;
			ss << "p2:  " << m2s[i] << std::endl;
			ss << "exp: " << out[i] << std::endl;
			ss << "act: " << pRel.getMode();
			throw std::runtime_error(ss.str());
		}
	}
}

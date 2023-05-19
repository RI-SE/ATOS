#include "dronecontrol.hpp"
#include "path.hpp"

using namespace ROSChannels;
using namespace std::placeholders;
using ObjectTrajectorySrv = atos_interfaces::srv::GetObjectTrajectory;

DroneControl::DroneControl() :
	Module(moduleName)
{
	// Entry-point to the module

	// Initialize services that this module provides to other modules
	objectTrajectoryService = create_service<ObjectTrajectorySrv>(ServiceNames::getObjectTrajectory,
		std::bind(&DroneControl::onRequestObjectTrajectory, this, _1, _2));
}

// Some module has requested the object trajectories
void DroneControl::onRequestObjectTrajectory(const ObjectTrajectorySrv::Request::SharedPtr req,
	const ObjectTrajectorySrv::Response::SharedPtr res)
{
	
	res->id = req->id;
	try {
		// TODO: Parse .path file
		auto path = createPath("path/path.path");

		// TODO: Create trajectory object for drone with id req->id
		auto traj = createDroneTrajectory(path,req->id);
		RCLCPP_INFO(get_logger(), "Esmini trajectory service called, returning trajectory for object %s", traj.toString().c_str());
		res->trajectory = traj.toCartesianTrajectory();
		res->success = true;
	}
	catch (std::out_of_range& e){
		RCLCPP_ERROR(get_logger(), "Esmini trajectory service called, no trajectory found for object %d", req->id);
		res->success = false;
	}
}

// Creates Path objects from .path file
ABD::Path DroneControl::createPath(const std::string& path)
{
	return ABD::Path(path);	
}
ATOS::Trajectory DroneControl::createDroneTrajectory(ABD::Path& path, uint32_t id)
{
	// TODO: Automate creation of multiple points
	auto traj = ATOS::Trajectory(get_logger());

	// The starting point
	auto point1 = ATOS::Trajectory::TrajectoryPoint(get_logger());
	point1.setXCoord(0);
	point1.setYCoord(0);
	point1.setZCoord(0);
	point1.setHeading(0);
	point1.setLongitudinalVelocity(3);
	point1.setLateralVelocity(3);
	point1.setLongitudinalAcceleration(3);
	point1.setLateralAcceleration(3);
	point1.setTime(0);
	traj.points.push_back(point1);

	// The ending point
	auto point2 = ATOS::Trajectory::TrajectoryPoint(get_logger());
	point2.setXCoord(1);
	point2.setYCoord(1);
	point2.setZCoord(1);
	point2.setHeading(0);
	point2.setLongitudinalVelocity(0);
	point2.setLateralVelocity(0);
	point2.setLongitudinalAcceleration(0);
	point2.setLateralAcceleration(0);
	point2.setTime(100);
	traj.points.push_back(point2);

	return traj;
}


DroneControl::~DroneControl() 
{
}

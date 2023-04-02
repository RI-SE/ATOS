#include "dronecontrol.hpp"
#include "roschannels/commandchannels.hpp"
#include "path.hpp"

using namespace ROSChannels;
using namespace std::placeholders;
using ObjectTrajectorySrv = atos_interfaces::srv::GetObjectTrajectory;

DroneControl::DroneControl() :
	Module(moduleName),
	initSub(*this, std::bind(&DroneControl::onInitMessage, this, _1))
{
	// Entry-point to the module

	// Initialize services that this module provides to other modules
	objectTrajectoryService = create_service<ObjectTrajectorySrv>(ServiceNames::getObjectTrajectory,
		std::bind(&DroneControl::onRequestObjectTrajectory, this, _1, _2));
}

// Object control requests the object trajectories
void DroneControl::onRequestObjectTrajectory(const ObjectTrajectorySrv::Request::SharedPtr request,
	const ObjectTrajectorySrv::Response::SharedPtr response)
{
	
}

// Creates Path objects from .path file
ABD::Path DroneControl::createPath(const std::string& path)
{
	
}

std::vector<ATOS::Trajectory> DroneControl::createDroneTrajectories(ABD::Path& path)
{
	// TODO: Create trajectory object(s) for drone(s)
	// droneTrajectories = somefunction(path);
}


DroneControl::~DroneControl() 
{
}

//! Init command callback
void DroneControl::onInitMessage(const Init::message_type::SharedPtr) 
{
	// Callback executed when this module receives a test-is-initialized command

	// TODO: Parse .path file
	auto path = createPath("path/path.path");

	// TODO: Create trajectory object(s) for drone(s)
	droneTrajectories = createDroneTrajectories(path);
}


#pragma once

#include <vector>
#include "trajectory.hpp"
#include "module.hpp"
#include "path.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"

/*!
 * \brief The DroneControl module 
 */
class DroneControl : public Module{
public:
	static inline std::string const moduleName = "drone_control";
	DroneControl();
	~DroneControl();

private:
	// Member variables
	std::map<uint32_t,ATOS::Trajectory> droneTrajectories; // map from drone object-id to trajectory

	// Provided services
	std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectTrajectory>> objectTrajectoryService;

	// Service callbacks
	void onRequestObjectTrajectory(const atos_interfaces::srv::GetObjectTrajectory::Request::SharedPtr,
		const atos_interfaces::srv::GetObjectTrajectory::Response::SharedPtr);

	// Business logic
	ABD::Path createPath(const std::string&);
	ATOS::Trajectory createDroneTrajectory(ABD::Path&, uint32_t);



};

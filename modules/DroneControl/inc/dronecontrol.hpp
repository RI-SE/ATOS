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
	std::vector<ATOS::Trajectory> droneTrajectories;

	// State subscribers
	ROSChannels::Init::Sub initSub;

	// State callbacks
	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;

	// Provided services
	std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectTrajectory>> objectTrajectoryService;

	// Service callbacks
	void onRequestObjectTrajectory(const atos_interfaces::srv::GetObjectTrajectory::Request::SharedPtr request,
		const atos_interfaces::srv::GetObjectTrajectory::Response::SharedPtr response);

	// Business logic
	ABD::Path createPath(const std::string& path);
	std::vector<ATOS::Trajectory> createDroneTrajectories(ABD::Path&);



};

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "ATOSbase.hpp"
#include "objectconfig.hpp"
#include <functional>

#include <ament_index_cpp/get_package_prefix.hpp>

using namespace ROSChannels;
using namespace std_srvs::srv;
using std::placeholders::_1, std::placeholders::_2;

ATOSBase::ATOSBase()
    : Module(ATOSBase::moduleName),
	exitSub(*this, std::bind(&ATOSBase::onExitMessage, this, _1))
{

	getObjectIdsService = create_service<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds,
		std::bind(&ATOSBase::onRequestObjectIDs, this, _1, _2));
}

ATOSBase::~ATOSBase()
{
}


void ATOSBase::onExitMessage(const Exit::message_type::SharedPtr)
{
    RCLCPP_INFO(get_logger(), "Received exit message");
    rclcpp::shutdown();
}

void ATOSBase::onRequestObjectIDs(
	const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request>,
	std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response> res)
{
	char path[PATH_MAX];
	std::vector<std::invalid_argument> errors;
	RCLCPP_INFO(get_logger(), "Received object ID information request");

	UtilGetObjectDirectoryPath(path, sizeof (path));
	fs::path objectDir(path);
	if (!fs::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	std::vector<uint32_t> objectIDs;
	for (const auto& entry : fs::directory_iterator(objectDir)) {
		if (!fs::is_regular_file(entry.status())) {
			continue;
		}

		ObjectConfig conf(get_logger());
		conf.parseObjectIdFromConfigurationFile(entry.path());

		RCLCPP_DEBUG(get_logger(), "Loaded configuration: %s", conf.toString().c_str());
		// Check preexisting
		auto foundID = std::find(objectIDs.begin(), objectIDs.end(), conf.getTransmitterID());
		if (foundID == objectIDs.end()) {
			objectIDs.push_back(conf.getTransmitterID());
		}
		else {
			std::string errMsg = "Duplicate object ID " + std::to_string(conf.getTransmitterID())
					+ " detected in object files";
			throw std::invalid_argument(errMsg);
		}
	}

	res->ids = objectIDs;
}
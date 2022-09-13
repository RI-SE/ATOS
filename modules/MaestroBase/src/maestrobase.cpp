#include "maestrobase.hpp"
#include "datadictionary.h"
#include "objectconfig.hpp"
#include <functional>

#include <ament_index_cpp/get_package_prefix.hpp>

using namespace ROSChannels;
using namespace std_srvs::srv;
using std::placeholders::_1, std::placeholders::_2;

MaestroBase::MaestroBase()
    : Module(MaestroBase::moduleName),
	exitSub(*this, std::bind(&MaestroBase::onExitMessage, this, _1))
{

	std::string maestroPath = ament_index_cpp::get_package_prefix("maestro");
	if (UtilVerifyTestDirectory(maestroPath.c_str()) == -1) {
        throw std::runtime_error("Failed to verify test directory");
  }


	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());
	initDataDictionaryService = create_service<SetBool>(ServiceNames::initDataDict,
		std::bind(&MaestroBase::onInitDataDictionary, this, _1, _2));
	getObjectIdsService = create_service<maestro_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds,
		std::bind(&MaestroBase::onRequestObjectIDs, this, _1, _2));
}

MaestroBase::~MaestroBase()
{
	auto result = DataDictionaryDestructor();
	if (result != WRITE_OK && result != READ_WRITE_OK) {
		RCLCPP_ERROR(get_logger(), "Unable to clear shared memory space");
	}
}

void MaestroBase::onInitDataDictionary(
	const SetBool::Request::SharedPtr req,
	SetBool::Response::SharedPtr res)
{
	RCLCPP_DEBUG(get_logger(), "Received request to initialize data dictionary, ignoring value %d", req->data);
	
	if (isInitialized) {
		res->message = "Data dictionary already initialized";
		res->success = true;
		return;
	}

	isInitialized = DataDictionaryConstructor() == READ_WRITE_OK;
	std::string message;
	if (isInitialized) {
		message = "Data dictionary successfully initialized";
		RCLCPP_INFO(get_logger(), message);
	}
	else {
		message = "Failed to initialize data dictionary";
		RCLCPP_ERROR(get_logger(), message);
		DataDictionaryDestructor();
	}
	res->success = isInitialized;
	res->message = message;
}

void MaestroBase::onExitMessage(const Exit::message_type::SharedPtr)
{
    RCLCPP_INFO(get_logger(), "Received exit message");
    auto result = DataDictionaryDestructor();
    if (result != WRITE_OK && result != READ_WRITE_OK) {
        RCLCPP_ERROR(get_logger(), "Unable to clear shared memory space");
    }
    rclcpp::shutdown();
}

void MaestroBase::onRequestObjectIDs(
	const std::shared_ptr<maestro_interfaces::srv::GetObjectIds::Request> req,
	std::shared_ptr<maestro_interfaces::srv::GetObjectIds::Response> res)
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

		ObjectConfig conf;
		conf.parseConfigurationFile(entry.path());

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
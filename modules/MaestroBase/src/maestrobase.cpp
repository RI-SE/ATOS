#include "maestrobase.hpp"
#include "datadictionary.h"
#include <functional>

MaestroBase::MaestroBase()
    : Module(MaestroBase::moduleName)
{
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());
	initDataDictionaryService = create_service<std_srvs::srv::SetBool>(ServiceNames::initDataDict,
		std::bind(&MaestroBase::onInitDataDictionary, this, std::placeholders::_1, std::placeholders::_2));

	exitSub = create_subscription<Empty>(TopicNames::exit, 0, bind(&MaestroBase::onExitMessage, this, _1));
}

MaestroBase::~MaestroBase()
{
	auto result = DataDictionaryDestructor();
	if (result != WRITE_OK && result != READ_WRITE_OK) {
		RCLCPP_ERROR(get_logger(), "Unable to clear shared memory space");
	}
}

void MaestroBase::initialize()
{
	if (UtilVerifyTestDirectory() == -1) {
        throw std::runtime_error("Failed to verify test directory");
    }
	RCLCPP_INFO(get_logger(), "Initializing data dictionary");
	auto result = DataDictionaryConstructor();
	if (result != READ_WRITE_OK) {
		DataDictionaryDestructor();
		throw std::runtime_error("Unable to initialize shared memory space");
	}
	else {
		RCLCPP_INFO(get_logger(), "Data dictionary succesfully initialized");
	}
}

void MaestroBase::onInitDataDictionary(
	const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
	std::shared_ptr<std_srvs::srv::SetBool::Response> res)
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

void MaestroBase::onExitMessage(const Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received exit message");
    auto result = DataDictionaryDestructor();
    if (result != WRITE_OK && result != READ_WRITE_OK) {
        RCLCPP_ERROR(get_logger(), "Unable to clear shared memory space");
    }
    rclcpp::shutdown();
}
#include "maestrobase.hpp"
#include "datadictionary.h"
#include <functional>

MaestroBase::MaestroBase()
    : Module(MaestroBase::moduleName)
{
	auto test = create_service<std_srvs::srv::SetBool>("init_data_dictionary",
		std::bind(&MaestroBase::onInitDataDictionary, this, std::placeholders::_1, std::placeholders::_2));
    initDataDictionaryService = test;

	exitSub = create_subscription<Empty>(topicNames[COMM_EXIT], 0, bind(&MaestroBase::onExitMessage, this, _1));
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
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());
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
		res->success = true;
		return;
	}

	auto result = DataDictionaryConstructor();
	res->success = result == READ_WRITE_OK;
	if (res->success) {
		DataDictionaryDestructor();
	}
	else {
		RCLCPP_INFO(get_logger(), "Data dictionary succesfully initialized");
		isInitialized = true;
	}
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
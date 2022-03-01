#include "maestrobase.hpp"
#include "datadictionary.h"

MaestroBase::MaestroBase()
    : Module(MaestroBase::moduleName)
{
    initialize();
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

void MaestroBase::onExitMessage(const Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received exit message");
    auto result = DataDictionaryDestructor();
    if (result != WRITE_OK && result != READ_WRITE_OK) {
        RCLCPP_ERROR(get_logger(), "Unable to clear shared memory space");
    }
    rclcpp::shutdown();
}
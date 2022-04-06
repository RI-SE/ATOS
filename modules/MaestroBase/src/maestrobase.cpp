#include "maestrobase.hpp"
#include "datadictionary.h"
#include <functional>

using namespace ROSChannels;
using namespace std_srvs::srv;
using std::placeholders::_1, std::placeholders::_2;

MaestroBase::MaestroBase()
    : Module(MaestroBase::moduleName),
	exitSub(*this, std::bind(&MaestroBase::onExitMessage, this, _1))
{
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());
	initDataDictionaryService = create_service<SetBool>(ServiceNames::initDataDict,
		std::bind(&MaestroBase::onInitDataDictionary, this, _1, _2));
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
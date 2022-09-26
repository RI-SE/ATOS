#pragma once

#include <future>
#include <exception>

#include "module.hpp"
#include "osi_handler.hpp"
#include "server.hpp"
#include "tcphandler.hpp"

#define DEFAULT_ADDRESS "127.0.0.1"
#define DEFAULT_PORT 55555
#define DEFAULT_DEBUG_VALUE false
#define QUALITY_OF_SERVICE 10
#define SEND_INTERVAL 500ms


class OSIAdapter : public Module
{
  public:
    void initialize(const TCPServer::Address address = DEFAULT_ADDRESS,
                   const TCPServer::Port port = DEFAULT_PORT,
                   bool debug = DEFAULT_DEBUG_VALUE);
    OSIAdapter();
    ~OSIAdapter();


  private:
    static inline std::string const moduleName = "osi_adapter";

    void sendOSIData();
    std::vector<char> makeOSIMessage(const OsiHandler::LocalObjectGroundTruth_t osiData);
    const OsiHandler::LocalObjectGroundTruth_t makeTestOSIData();
    
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    TCPServer tcp;
    Socket connection;
};
#pragma once

#include <boost/asio.hpp>

#include "module.hpp"
#include "osi_handler.hpp"

#define DEFAULT_ADDRESS "127.0.0.1"
#define DEFAULT_PORT 55555
#define DEFAULT_DEBUG_VALUE false
#define QUALITY_OF_SERVICE 10
#define SEND_INTERVAL 500ms

class OSIAdapter : public Module
{
  public:
    void initialize(const std::string& address = DEFAULT_ADDRESS,
                  const uint16_t port = DEFAULT_PORT,
                  const bool debug = DEFAULT_DEBUG_VALUE);
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

    boost::asio::ip::tcp::endpoint endpoint;
    std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
    std::shared_ptr<boost::asio::io_service> io_service;
    std::shared_ptr<boost::asio::ip::tcp::socket> socket;

    void setupTCPServer(boost::asio::ip::tcp::endpoint endpoint);
    void destroyTCPServer();
    void resetTCPServer(boost::asio::ip::tcp::endpoint endpoint);
};

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pointcloudpublisher.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>


/**
 * @brief PointcloudPublisher constructor.
 * 
 */
PointcloudPublisher::PointcloudPublisher() : Module(PointcloudPublisher::moduleName),
  initSub(*this, std::bind(&PointcloudPublisher::onInitMessage, this, std::placeholders::_1))
{
  declare_parameter("pointcloud_files");
}


/**
 * @brief PointcloudPublisher destructor.
 * 
 */
PointcloudPublisher::~PointcloudPublisher() {}


/**
 * @brief Get pointcloud-file, load pointcloud-file, create a pointcloud-message.
 * 
 */
void PointcloudPublisher::initialize() {
  pointclouds.clear();
  pointcloudFiles.clear();
  pointcloudPubs.clear();

  getPointcloudFiles();
  loadPointClouds();
  createPublishers();
}


/**
 * @brief Get the path to the pointcloud-file.
 * 
 */
void PointcloudPublisher::getPointcloudFiles() {
  get_parameter("pointcloud_files", pointcloudFiles);
  const std::string homeDir = getenv("HOME");
  for (auto& pointcloudFile : pointcloudFiles) {
    pointcloudFile = homeDir + "/.astazero/ATOS/pointclouds/" + pointcloudFile;
  }
}


/**
 * @brief Load the pointcloud-file, throws std::runtime_error if can't find file.
 * 
 */
void PointcloudPublisher::loadPointClouds() {
  for (auto& pointcloudFile : pointcloudFiles) {
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointcloudFile, *pointcloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Could not read file %s", pointcloudFile.c_str());
    }
    else {
      pointclouds[pointcloudFile] = pointcloud;
      RCLCPP_INFO(get_logger(), "Loaded pointcloud %s with %d points", pointcloudFile.c_str(), pointcloud->width * pointcloud->height);
    }
  }
}


/**
 * @brief Create pointcloud publishers
 * 
 */
void PointcloudPublisher::createPublishers() {
  for (auto& pointcloudFile : pointcloudFiles) {
    auto pointcloudPub = std::make_shared<ROSChannels::Pointcloud::Pub>(*this, getPublisherTopicName(pointcloudFile));
    pointcloudPubs[pointcloudFile] = pointcloudPub;
  }
}


/**
 * @brief Returns the topic name from a path
 * 
 * @param path Path to make topicn ame from
 * @return std::string The topic name
 */
std::string PointcloudPublisher::getPublisherTopicName(const std::string& path) {
  auto str = path.substr(path.rfind('/') + 1); 
  return str.substr(0, str.length() - 4);
}


/**
 * @brief Publish pointcloud on init message
 * 
 */
void PointcloudPublisher::onInitMessage(const ROSChannels::Init::message_type::SharedPtr) {
  initialize();

  for (auto& pointcloudFile : pointcloudFiles) {
    sensor_msgs::msg::PointCloud2 msg;

    auto pointcloud = pointclouds[pointcloudFile];
    pcl::toROSMsg(*pointcloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();

    auto pointcloudPub = pointcloudPubs[pointcloudFile];
    pointcloudPub->publish(msg);
  }
}

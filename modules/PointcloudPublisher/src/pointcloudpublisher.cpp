/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pointcloudpublisher.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>


/**
 * @brief PointcloudPublisher constructor.
 * 
 */
PointcloudPublisher::PointcloudPublisher() : Module(PointcloudPublisher::moduleName),
  initSub(*this, std::bind(&PointcloudPublisher::onInitMessage, this, std::placeholders::_1)),
  pointcloudPub(*this)
{
  initialize();
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
  getPointcloudFile();
  loadPointCloud();
  createPointcloudMessage();
}


/**
 * @brief Get the path to the pointcloud-file.
 * 
 */
void PointcloudPublisher::getPointcloudFile() {
  declare_parameter("pointcloud_file");
  get_parameter("pointcloud_file", pointcloudFile);
  const std::string homeDir = getenv("HOME");
  pointcloudFile = homeDir + "/.astazero/ATOS/pointclouds/" + pointcloudFile;
}


/**
 * @brief Load the pointcloud-file, throws std::runtime_error if can't find file.
 * 
 */
void PointcloudPublisher::loadPointCloud() {
  pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointcloudFile, *pointcloud) == -1) {
      RCLCPP_ERROR(get_logger(), "Could not read file %s", pointcloudFile.c_str());
      throw std::runtime_error("Could not open file");
  }
  RCLCPP_INFO(get_logger(), "Loaded pointcloud with %d points", pointcloud->width * pointcloud->height);
}


/**
 * @brief Creates a sensor_msgs::msg::PointCloud2 message and fills it with data.
 * 
 */
void PointcloudPublisher::createPointcloudMessage() {
  pcl::toROSMsg(*pointcloud, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
  
}


/**
 * @brief Publish pointcloud on init message
 * 
 */
void PointcloudPublisher::onInitMessage(const ROSChannels::Init::message_type::SharedPtr) {
  pointcloudPub.publish(msg);
}

// TODO::
// iterate over input files, publish several pointclouds
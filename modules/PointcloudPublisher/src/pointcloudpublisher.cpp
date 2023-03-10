/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pointcloudpublisher.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>


using namespace std::chrono_literals;

PointcloudPublisher::PointcloudPublisher() : Module(PointcloudPublisher::moduleName) {
  initialize();
  RCLCPP_INFO(get_logger(), "POINTCLOUD FILE: %s", pointcloudFile.c_str());

  publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/atos/pointcloud", 1);
  timer = this->create_wall_timer(500ms, std::bind(&PointcloudPublisher::publishPointcloud, this));

}

PointcloudPublisher::~PointcloudPublisher() {}


void PointcloudPublisher::initialize() {
  getPointcloudFile();
  loadPointCloud();
  createPointcloudMessage();
}

void PointcloudPublisher::getPointcloudFile() {
  declare_parameter("pointcloud_file");
  get_parameter("pointcloud_file", pointcloudFile);
  const std::string homeDir = getenv("HOME");
  pointcloudFile = homeDir + "/.astazero/ATOS/pointclouds/" + pointcloudFile;
}

void PointcloudPublisher::loadPointCloud() {
  pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointcloudFile, *pointcloud) == -1) {
      RCLCPP_ERROR(get_logger(), "Could not read file %s", pointcloudFile.c_str());
      // throw exception here
  }
  RCLCPP_INFO(get_logger(), "POINT CLOUD: width %d height %d", pointcloud->width, pointcloud->height);
}


void PointcloudPublisher::createPointcloudMessage() {
  pcl::toROSMsg(*pointcloud, msg);

  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
  
}


void PointcloudPublisher::publishPointcloud() {
  publisher->publish(msg);
}
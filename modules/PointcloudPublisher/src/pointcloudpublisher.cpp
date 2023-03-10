/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pointcloudpublisher.hpp"
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
  createPointcloudMessage();
  loadPointCloud();
}

void PointcloudPublisher::getPointcloudFile() {
  declare_parameter("pointcloud_file");
  get_parameter("pointcloud_file", pointcloudFile);
  const std::string homeDir = getenv("HOME");
  pointcloudFile = homeDir + "/.astazero/ATOS/pointclouds/" + pointcloudFile;
}

void PointcloudPublisher::loadPointCloud() {
  
}


void PointcloudPublisher::createPointcloudMessage() {
  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
}


void PointcloudPublisher::publishPointcloud() {
  publisher->publish(msg);
}
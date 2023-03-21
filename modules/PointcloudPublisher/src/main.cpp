/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <iostream>
#include "pointcloudpublisher.hpp"

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  auto pointcloudPublisherNode = std::make_shared<PointcloudPublisher>();
  rclcpp::spin(pointcloudPublisherNode);
  rclcpp::shutdown();

  return 0;
}
/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "roschannels/pointcloudchannel.hpp"
#include "roschannels/commandchannels.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


class PointcloudPublisher : public Module {

  public:
    PointcloudPublisher();
    ~PointcloudPublisher();

  private:
    static inline std::string const moduleName = "pointcloud_publisher";
    ROSChannels::Init::Sub initSub;
    std::map<std::string, std::shared_ptr<ROSChannels::Pointcloud::Pub>> pointcloudPubs;

    std::vector<std::string> pointcloudFiles;
    std::map<std::string, std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> pointclouds;

    void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
    void initialize();
    void readPointcloudParams();
    void loadPointClouds();
    void createPublishers();
    std::string getPublisherTopicName(const std::string& path) const;

};
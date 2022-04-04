#include "trajectorypublisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/path.hpp>

using namespace maestro;

TrajectoryPublisher::TrajectoryPublisher(
    rclcpp::Node& node,
    const Trajectory& traj,
    const uint32_t objectId)
    : traj(std::make_shared<Trajectory>(traj)),
    pub(node,objectId),
    objectId(objectId)
{
    beginPublish();
}

void TrajectoryPublisher::handleStart() {
    using std::chrono::steady_clock;
    startTime.reset(new steady_clock::time_point(steady_clock::now()));
}


nav_msgs::msg::Path TrajectoryPublisher::extractChunk() {
    using std::chrono::steady_clock;

    nav_msgs::msg::Path ret;
    auto firstPoint = traj->points.begin();
    auto now = steady_clock::now();
    auto diff = steady_clock::duration(0);
    if (startTime) {
        diff = now-*startTime;
        firstPoint = std::upper_bound(traj->points.begin(), traj->points.end(), diff,
            [](const steady_clock::duration& dur, const Trajectory::TrajectoryPoint& pt) {
                return pt.getTime() > dur;
            });
        if (firstPoint == traj->points.end()) {
            return ret;
        }
    }
    auto endTime = diff + chunkLength;
    auto lastPoint = std::upper_bound(firstPoint, traj->points.end(), endTime,
        [](const steady_clock::duration& dur, const Trajectory::TrajectoryPoint& pt) {
            return pt.getTime() > dur;
        }
    );
    
    auto firstPointTime = rclcpp::Time(
        (firstPoint->getTime() + (startTime ? *startTime : now))
        .time_since_epoch().count());

    ret.header.frame_id = "map";
    ret.header.stamp = firstPointTime;

    std::transform(firstPoint, lastPoint, std::back_inserter(ret.poses),
        [&](const Trajectory::TrajectoryPoint& pt){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = ret.header.frame_id;
            pose.header.stamp = firstPointTime + rclcpp::Duration(pt.getTime());
            pose.pose.position.x = pt.getXCoord();
            pose.pose.position.y = pt.getYCoord();
            pose.pose.position.z = pt.getZCoord();
            tf2::Quaternion q;
            q.setRPY(0, 0, pt.getHeading());
            tf2::convert(q, pose.pose.orientation);
            return pose;
        }
    );
    return ret;
}

void TrajectoryPublisher::beginPublish() {
    pubThread = std::thread([this]() {
        while (true) {
            auto wakeTime = std::chrono::steady_clock::now() + publishPeriod;
            auto trajMsg = extractChunk();
            pub.publish(trajMsg);
            std::this_thread::sleep_until(wakeTime);
        }
    });
}
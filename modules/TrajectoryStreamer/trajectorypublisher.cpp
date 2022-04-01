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

}

void TrajectoryPublisher::beginPublish() {
    pubThread = std::thread([this]() {
        while (true) {
            nav_msgs::msg::Path trajMsg;
            trajMsg.header.stamp = rclcpp::Clock().now();
            trajMsg.header.frame_id = "map";
            for (const auto& trajPt : traj->points) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = trajMsg.header.stamp; // TODO
                pose.header.frame_id = trajMsg.header.frame_id;

                pose.pose.position.x = trajPt.getXCoord();
                pose.pose.position.y = trajPt.getYCoord();
                try {
                    pose.pose.position.z = trajPt.getZCoord();
                } catch (std::exception&) {
                    pose.pose.position.z = 0;
                }
		        tf2::Quaternion orientation;
		        orientation.setRPY(0, 0, trajPt.getHeading());
		        pose.pose.orientation = tf2::toMsg(orientation);
                trajMsg.poses.push_back(pose);
            }
            pub.publish(trajMsg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
}
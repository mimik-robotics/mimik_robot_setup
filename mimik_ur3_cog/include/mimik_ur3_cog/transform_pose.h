#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace mimik {
namespace ur3_cog {

geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose& from_offset, const std::string& from_frame,
                                         const std::string& to_frame);

geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose& from_offset, const std::string& from_frame,
                                         const std::string& to_frame, planning_scene::PlanningScene& scene);

}  // namespace ur3_cog
}  // namespace mimik

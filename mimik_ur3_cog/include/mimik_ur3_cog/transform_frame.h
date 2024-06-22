#pragma once

#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <moveit/planning_scene/planning_scene.h>

namespace mimik {
namespace ur3_cog {

/** Converts a frame into the desired frame
 *
 * Uses MoveIt PlanningScene for conversion.
 *
 * Minimal initialization example:
 *   ```cpp
 *   auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
 *   auto tf_listener = tf2_ros::TransformListener(*tf_buffer);
 *
 *   planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf_buffer);
 *   planning_scene::PlanningScene& scene{ *psm.getPlanningScene() };
 *   ```
 */
geometry_msgs::TransformStamped transformFrame(const std::string& from_frame, const std::string& to_frame,
                                               const planning_scene::PlanningScene& scene);

geometry_msgs::TransformStamped transformFrame(const std::string& from_frame, const Eigen::Isometry3d& from_offset,
                                               const std::string& to_frame, const planning_scene::PlanningScene& scene);

}  // namespace ur3_cog
}  // namespace mimik

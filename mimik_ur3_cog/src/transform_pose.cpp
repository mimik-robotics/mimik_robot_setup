#include <mimik_ur3_cog/transform_pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.h>

#include <stdexcept>

namespace mimik {
namespace ur3_cog {

geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose& from_offset, const std::string& from_frame,
                                         const std::string& to_frame, planning_scene::PlanningScene& scene)
{
  if (!scene.knowsFrameTransform(from_frame))
    throw std::runtime_error("Frame '" + from_frame + "' not known in planning scene");

  if (!scene.knowsFrameTransform(to_frame))
    throw std::runtime_error("Frame '" + to_frame + "' not known in planning scene");

  // compute global -> from_frame + offset
  auto tf_global_from_offset = scene.getFrameTransform(from_frame);
  Eigen::Isometry3d from_offset_eig = Eigen::Isometry3d::Identity();
  tf2::fromMsg(from_offset, from_offset_eig);

  tf_global_from_offset = tf_global_from_offset * from_offset_eig;

  // compute global -> to_frame
  auto tf_global_to = scene.getFrameTransform(to_frame);

  // compute to_frame -> from_frame + offset
  auto tf_to_from_offset = tf_global_to.inverse() * tf_global_from_offset;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = to_frame;
  pose_stamped.pose = tf2::toMsg(tf_to_from_offset);

  return pose_stamped;
}

geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose& from_offset, const std::string& from_frame,
                                         const std::string& to_frame)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  auto tf_listener = tf2_ros::TransformListener(*tf_buffer);

  planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf_buffer);
  planning_scene::PlanningScene& scene{ *psm.getPlanningScene() };

  return transformPose(from_offset, from_frame, to_frame, scene);
}

}  // namespace ur3_cog
}  // namespace mimik

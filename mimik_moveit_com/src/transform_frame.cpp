#include <mimik_moveit_com/transform_frame.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_eigen/tf2_eigen.h>

#include <stdexcept>

namespace mimik_moveit_com {

geometry_msgs::TransformStamped transformFrame(const std::string& from_frame, const std::string& to_frame,
                                               const planning_scene::PlanningScene& scene)
{
  return transformFrame(from_frame, Eigen::Isometry3d::Identity(), to_frame, scene);
}

geometry_msgs::TransformStamped transformFrame(const std::string& from_frame, const Eigen::Isometry3d& from_offset,
                                               const std::string& to_frame, const planning_scene::PlanningScene& scene)
{
  if (!scene.knowsFrameTransform(from_frame))
    throw std::runtime_error("Frame '" + from_frame + "' not known in planning scene");

  if (!scene.knowsFrameTransform(to_frame))
    throw std::runtime_error("Frame '" + to_frame + "' not known in planning scene");

  // compute global -> from_frame + offset
  auto tf_global_from_offset = scene.getFrameTransform(from_frame);

  tf_global_from_offset = tf_global_from_offset * from_offset;

  // compute global -> to_frame
  auto tf_global_to = scene.getFrameTransform(to_frame);

  // compute to_frame -> from_frame + offset
  auto tf_to_from_offset = tf_global_to.inverse() * tf_global_from_offset;

  geometry_msgs::TransformStamped transform_stamped = tf2::eigenToTransform(tf_to_from_offset);

  transform_stamped.header.frame_id = to_frame;

  return transform_stamped;
}

}  // namespace mimik_moveit_com

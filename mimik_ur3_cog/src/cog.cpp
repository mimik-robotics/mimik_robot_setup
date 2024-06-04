#include <mimik_ur3_cog/cog.h>
#include <mimik_ur3_cog/transform_pose.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <iostream>

namespace mimik {
namespace ur3_cog {

COG computeCOG(const std::vector<COG>& cog_vec)
{
  COG cog_total;

  Eigen::Vector3d cog_num = Eigen::Vector3d{ 0, 0, 0 };
  double cog_den = 0;

  for (const auto& cog : cog_vec)
  {
    cog_num += cog.mass * cog.cog;
    cog_den += cog.mass;
  }

  cog_total.cog = cog_num / cog_den;
  cog_total.mass = cog_den;

  return cog_total;
}

COG computeCOG(const std::vector<geometry_msgs::InertiaStamped>& inertia_vec, const std::string& to_frame)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  auto tf_listener = tf2_ros::TransformListener(*tf_buffer);

  planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf_buffer);
  planning_scene::PlanningScene& scene{ *psm.getPlanningScene() };

  std::vector<COG> cog_vec;
  cog_vec.resize(inertia_vec.size());

  for (const auto& inertia : inertia_vec)
  {
    geometry_msgs::Pose pose_offset;
    pose_offset.position.x = inertia.inertia.com.x;
    pose_offset.position.y = inertia.inertia.com.y;
    pose_offset.position.z = inertia.inertia.com.z;
    pose_offset.orientation.w = 1.0;

    geometry_msgs::PoseStamped pose;
    pose = transformPose(pose_offset, inertia.header.frame_id, to_frame, scene);

    cog_vec.emplace_back(
        COG{ .mass = inertia.inertia.m,
             .cog = Eigen::Vector3d{ pose.pose.position.x, pose.pose.position.y, pose.pose.position.z } });
  }

  // compute CoG
  return computeCOG(cog_vec);
}

void COGToInertia(const COG& cog, geometry_msgs::Inertia& inertia)
{
  inertia.com.x = cog.cog.x();
  inertia.com.y = cog.cog.y();
  inertia.com.z = cog.cog.z();

  inertia.m = cog.mass;
}

void print(const COG& cog)
{
  std::cout << "mass  = " << cog.mass << std::endl;
  std::cout << "cog_x = " << cog.cog.x() << std::endl;
  std::cout << "cog_y = " << cog.cog.y() << std::endl;
  std::cout << "cog_z = " << cog.cog.z() << std::endl;
}

}  // namespace ur3_cog
}  // namespace mimik

#include <mimik_moveit_com/com.h>
#include <mimik_moveit_com/transform_frame.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <iostream>

namespace mimik_moveit_com {

CoM computeCoM(const std::vector<CoM>& com_vec)
{
  CoM com_total;

  Eigen::Vector3d com_num = Eigen::Vector3d{ 0, 0, 0 };
  double com_den = 0;

  for (const auto& com : com_vec)
  {
    com_num += com.mass * com.com;
    com_den += com.mass;
  }

  com_total.com = com_num / com_den;
  com_total.mass = com_den;

  return com_total;
}

CoM computeCoM(const std::vector<geometry_msgs::InertiaStamped>& inertia_vec, const std::string& to_frame)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  auto tf_listener = tf2_ros::TransformListener(*tf_buffer);

  planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf_buffer);
  planning_scene::PlanningScene& scene{ *psm.getPlanningScene() };

  std::vector<CoM> com_vec;
  com_vec.resize(inertia_vec.size());

  for (const auto& inertia : inertia_vec)
  {
    Eigen::Isometry3d eig_offset = Eigen::Isometry3d::Identity();
    eig_offset.translate(Eigen::Vector3d(inertia.inertia.com.x, inertia.inertia.com.y, inertia.inertia.com.z));

    geometry_msgs::TransformStamped tf;
    tf = transformFrame(inertia.header.frame_id, eig_offset, to_frame, scene);

    com_vec.emplace_back(CoM{
        .mass = inertia.inertia.m,
        .com = Eigen::Vector3d{ tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z } });
  }

  // compute Com
  return computeCoM(com_vec);
}

void CoMToInertia(const CoM& com, geometry_msgs::Inertia& inertia)
{
  inertia.com.x = com.com.x();
  inertia.com.y = com.com.y();
  inertia.com.z = com.com.z();

  inertia.m = com.mass;
}

void print(const CoM& com)
{
  std::cout << "mass  = " << com.mass << std::endl;
  std::cout << "com_x = " << com.com.x() << std::endl;
  std::cout << "com_y = " << com.com.y() << std::endl;
  std::cout << "com_z = " << com.com.z() << std::endl;
}

}  // namespace mimik_moveit_com

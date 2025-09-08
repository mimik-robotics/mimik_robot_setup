#pragma once

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>

#include <Eigen/Geometry>

namespace mimik_moveit_com {

struct CoM
{
  double mass;          // kg
  Eigen::Vector3d com;  // meters
};

/*
 *    Center of gravity equation:
 *
 *      n
 *     ___
 *     ╲    m  ⋅ d
 *     ╱     i    i
 *     ‾‾‾
 *    i = 1
 *    ─────────────
 *        n
 *       ___
 *       ╲    m
 *       ╱     i
 *       ‾‾‾
 *      i = 1
 */
CoM computeCoM(const std::vector<CoM>& cog_vec);

CoM computeCoM(const std::vector<geometry_msgs::InertiaStamped>& inertia_vec, const std::string& to_frame);

void CoMToInertia(const CoM& cog, geometry_msgs::Inertia& inertia);

void print(const CoM& cog);

}  // namespace mimik_moveit_com

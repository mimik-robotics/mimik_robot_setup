#pragma once

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>

#include <Eigen/Geometry>

namespace mimik {
namespace ur3_cog {

struct COG
{
  double mass;          // kg
  Eigen::Vector3d cog;  // meters
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
COG computeCOG(const std::vector<COG>& cog_vec);

COG computeCOG(const std::vector<geometry_msgs::InertiaStamped>& inertia_vec, const std::string& to_frame);

void COGToInertia(const COG& cog, geometry_msgs::Inertia& inertia);

void print(const COG& cog);

}  // namespace ur3_cog
}  // namespace mimik

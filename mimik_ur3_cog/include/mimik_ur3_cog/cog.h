#pragma once

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

void print(const COG& cog);

}  // namespace ur3_cog
}  // namespace mimik

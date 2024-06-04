#include <mimik_ur3_cog/cog.h>

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

void print(const COG& cog)
{
  std::cout << "mass  = " << cog.mass << std::endl;
  std::cout << "cog_x = " << cog.cog.x() << std::endl;
  std::cout << "cog_y = " << cog.cog.y() << std::endl;
  std::cout << "cog_z = " << cog.cog.z() << std::endl;
}

}  // namespace ur3_cog
}  // namespace mimik

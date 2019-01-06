#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::AngleAxis<double> rot1(xzx[0] * M_PI / 180.0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxis<double> rot2(xzx[1] * M_PI / 180.0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxis<double> rot3(xzx[2] * M_PI / 180.0, Eigen::Vector3d::UnitX());

  return  Eigen::Affine3d(rot3 * rot2 * rot1);
  /////////////////////////////////////////////////////////////////////////////
}

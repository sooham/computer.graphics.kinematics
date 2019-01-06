#include "transformed_tips.h"
#include "forward_kinematics.h"
#include <cassert>
#include <iostream>
#include <limits>

typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> TransformVector;

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  TransformVector T;
  // compute forward kinematics
  forward_kinematics(skeleton, T);

  Eigen::VectorXd tips(b.size()*3);

  for (int i=0; i<b.size(); i++) {
     // compute the tip location for bone b[i]
     Bone bone = skeleton[b[i]];
     Eigen::Vector4d tip_b_h = T[b[i]] * bone.rest_T * Eigen::Vector4d(bone.length, 0, 0, 1);

     tips.segment(i*3, 3) = tip_b_h.head(3) / tip_b_h(3);

  }

  return tips;
}


#include "copy_skeleton_at.h"
#include <cassert>
#include <iostream>
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  Skeleton new_skeleton = skeleton;
  for (int i = 0; i < skeleton.size(); ++i) {
    new_skeleton[i].xzx = A.segment(i*3, 3);
  }
  return new_skeleton;
}

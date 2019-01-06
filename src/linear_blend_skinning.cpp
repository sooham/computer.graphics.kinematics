#include "linear_blend_skinning.h"
#include <iostream>

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
    U.resize(V.rows(), 3);

    for (int vi=0; vi<V.rows(); vi++) {
        // for each vertex
        // compute linear combination over all bones of transformed vertex
        Eigen::RowVector3d v = V.row(vi);

        Eigen::Vector3d pose_v = Eigen::Vector3d::Zero();

        for (int bi=0; bi < skeleton.size(); bi++) {
            if (skeleton[bi].weight_index != -1) { 
                pose_v += W(vi, skeleton[bi].weight_index) * (T[bi].linear() * v.transpose() + T[bi].translation());
            }
        }

        U.row(vi) = pose_v.transpose();
    }
}

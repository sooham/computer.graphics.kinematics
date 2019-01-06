#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>
#include <cmath>
#include <cassert>

void end_effectors_objective_and_gradient(
  // skeleton
  const Skeleton & skeleton,
  // index into skeleton for end effector constraints
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code


  // compute least squares objective given bones list of euler angles
  f = [&](const Eigen::VectorXd & A)->double
  {
    // input A - #bones * 3 list of euler angles
    double total = 0;

    // copy skeleton, apply angles, get tips
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tips = transformed_tips(copy, b);

    // xb0 is position of tips of b
    for (int i=0; i<b.size(); i++) {
        total += (tips.segment(3*i, 3) - xb0.segment(3*i, 3)).squaredNorm();
    }
    return total;
  };

  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    // copy skeleton, apply angles, get tips
    Skeleton copy = copy_skeleton_at(skeleton, A);
    // end effector tips
    Eigen::VectorXd tips = transformed_tips(copy, b);

    // get jacobian
    Eigen::MatrixXd J;
    kinematics_jacobian(copy, b, J);

    // get the gradient, which is a sum over linear combination of rows of the jacobian
    Eigen::VectorXd grad = Eigen::MatrixXd::Zero(A.rows(), A.cols());
    for (int bi=0;bi<b.size();bi++) {
        // for each end effector
        for (int i=0;i<3;i++) {
                // for each index, compute the gradient
                grad += 2 * (tips(3*bi+i) - xb0(3*bi+i)) *  J.row(3*bi+i).transpose();
        }
    }

    return grad;
  };

  proj_z = [&](Eigen::VectorXd & A)
  {
    assert(skeleton.size()*3 == A.size());
    for (int i=0;i<skeleton.size();i++) {
        for (int j=0;j<3;j++) {
            A(i*3+j) = std::min(skeleton[i].xzx_max(j), std::max(skeleton[i].xzx_min(j), A(i*3+j)));
        }
    }
  };
  /////////////////////////////////////////////////////////////////////////////
}

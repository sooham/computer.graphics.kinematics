#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>
#include <cassert>

void kinematics_jacobian(
  // skeleton
  const Skeleton & skeleton,
  // indexes into skeleton for end-effector bones
  const Eigen::VectorXi & b,
  // output: len(b)*3, skeleton*3 matrix
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  // assume the euler angle rotation transform has been applied to each
  // bone in the skeleton, use forward differencing

  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  Eigen::MatrixXd M = J;

  // compute tip positions for current joint angles
  Eigen::VectorXd ef_tips = transformed_tips(skeleton, b);
  Skeleton copy = skeleton;

  for (int ei=0; ei<b.size();ei++) {
    // for each end-effector
    // get the tip of the end effector
    Eigen::Vector3d ef_tip = ef_tips.segment(3*ei, 3);

    for (int bi=0;bi<skeleton.size();bi++) {
            // for each bone in skeleton adjust bone and
            // compute new position
            Eigen::Vector3d orig_xzx = skeleton[bi].xzx;
            for (int ai = 0; ai<3; ai++) { // for angles
                copy[bi].xzx(ai) += 1e-7;
                Eigen::Vector3d new_ef_tip = transformed_tips(copy, b).segment(ei*3, 3);
                Eigen::Vector3d delta = (new_ef_tip - ef_tip) / 1e-7;

                // update Jacobian
                for (int ci=0;ci<3;ci++) {
                    J(ei*3+ci,bi*3+ai) = delta(ci);
                }
                copy[bi].xzx = orig_xzx;
            }
    }
  }
  /////////////////////////////////////////////////////////////////////////////
}

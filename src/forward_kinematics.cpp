#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function
#include <iostream>

typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> TransformVector;

void rigid_pose_transform(int idx, const Skeleton & skeleton, std::vector<bool> & tracker, TransformVector & T);

void forward_kinematics(
  const Skeleton & skeleton,
  TransformVector & T)
{
  // compute deformations for each node in a forward kinematics hierarchy

  // resize t to skeleton size
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  // vector to keep track of computed rest -> pose transformations
  std::vector<bool> computed_rigid_pose_transform(skeleton.size(), false);

  for (int i=0; i<skeleton.size();++i) {
      rigid_pose_transform(i, skeleton, computed_rigid_pose_transform, T);
  }

  /*
  for (int i=0; i<skeleton.size();++i) {
     std::cout << "Bone number " << i+1 << "\n";
     std::cout << "\t parent idx: " << skeleton[i].parent_index << "\n";
     std::cout << "\t length " << skeleton[i].length << "\n";
     std::cout << "\t rest T \n" << skeleton[i].rest_T.matrix() << "\n";
     std::cout << "\t xzx\n" << skeleton[i].xzx << "\n";
     std::cout << "\t rigid pose T\n" << T[i].matrix() << "\n";
     std::cout << "-----------------------------\n";
  }
  */
}

// Recursively compute the transformation for bone = skeletion[idx] and add
// to tracker
void rigid_pose_transform(int idx, const Skeleton & skeleton, std::vector<bool> & tracker, TransformVector & T) { 
        if (tracker[idx]) {
            // base case: rigid pose transform was previously computed 
            return;
        }

        Bone b = skeleton[idx];
        // base case: root bone input
        if (b.parent_index == -1) {
            T[idx] = Eigen::Affine3d::Identity();
        } else {
            // recursive case
            // first, compute the Transformation matrix from euler angles
            Eigen::Affine3d rot_T = euler_angles_to_transform(b.xzx);

            if (!tracker[b.parent_index]) {
                // recurse and get rigid transformation for parent
                rigid_pose_transform(b.parent_index, skeleton, tracker, T);
            }
            T[idx] = T[b.parent_index] * b.rest_T * rot_T * b.rest_T.inverse();
        }
        tracker[idx] = true;
}

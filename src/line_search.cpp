#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  /////////////////////////////////////////////////////////////////////////////

  double step_sz = max_step;
  int num_steps = 0;

  double e_val = f(z);

  while (num_steps < 20) {
     Eigen::VectorXd new_z = z;
     new_z -= step_sz * dz;
     proj_z(new_z);
     if (e_val > f(new_z)) {
        return step_sz;
     }
     step_sz /= 2;
     num_steps++;
  }
  return step_sz;
}

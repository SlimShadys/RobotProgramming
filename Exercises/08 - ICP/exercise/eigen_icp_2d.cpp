#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include "Eigen/Cholesky"
#include <iostream>

using namespace std;

ICP::ICP(const ContainerType &fixed_,
         const ContainerType &moving_,
         int min_points_in_leaf) : _fixed(fixed_),
                                   _moving(moving_),
                                   _kd_tree(_fixed.begin(), _fixed.end(), min_points_in_leaf) {}

void ICP::computeCorrespondences()
{
  _correspondences.clear(); // Clear any previous correspondences

  // We loop for every point in _moving
  for (const auto &m : _moving)
  {
    Vector2f m_prime = _X * m; // Compute the transformation m' = _X * m

    // Find the nearest neighbor in the fixed set _fixed for the transformed point m_prime
    Vector2f *nearest_fixed = _kd_tree.bestMatchFast(m_prime, _ball_radius);

    if (nearest_fixed)
    {
      // We construct a PointPair using the found nearest fixed point and m_prime and
      // then we add the correspondence pair to the list of correspondences
      _correspondences.push_back(PointPair(*nearest_fixed, m_prime));
    }
  }
}

void ICP::optimizeCorrespondences()
{
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b;
  H.setZero();
  b.setZero();
  Eigen::Matrix<float, 2, 3> J;
  J.block<3, 3>(0, 0).setIdentity();
  _num_kernelized = 0;
  _num_inliers = 0;
  _chi2_sum = 0;
  for (const auto &c : _correspondences)
  {
    const auto &f = c._fixed;
    const auto &m = c._moving;
    J(0, 2) = -m.y();
    J(1, 2) = m.x();
    Vector2f e = m - f;
    float scale = 1;
    float chi = e.squaredNorm();
    _chi2_sum += chi;
    if (e.squaredNorm() > _kernel_chi2)
    {
      scale = sqrt(_kernel_chi2 / chi);
      ++_num_kernelized;
    }
    else
    {
      ++_num_inliers;
    }
    H.noalias() += scale * J.transpose() * J;
    b.noalias() += scale * J.transpose() * e;
  }
  _dx = H.ldlt().solve(-b);
  Eigen::Isometry2f dX;
  const Eigen::Matrix2f dR = Rtheta(_dx(2));
  dX.setIdentity();
  dX.linear() = dR;
  dX.translation() = _dx.block<2, 1>(0, 0);
  _X = dX * _X;
}

void ICP::run(int max_iterations, Eigen::Isometry2f iso)
{
  int current_iteration = 0;
  while (current_iteration < max_iterations)
  {
    computeCorrespondences();
    optimizeCorrespondences();
    ++current_iteration;
    cout << "Iteration: " << current_iteration;
    cout << " | Correspondences: " << numCorrespondences();
    cout << " | Inliers: " << numInliers();
    cout << " | Kernelized: " << numKernelized();
    cout << " | Chi: " << _chi2_sum << endl;
  }
  cout << "------------------------------------" << endl;
  cout << "Starting Transformation Matrix: \n" << iso.matrix() << endl;
  cout << "------------------------------------" << endl;
  cout << "Final estimated Matrix: \n" << _X.matrix() << endl;
}

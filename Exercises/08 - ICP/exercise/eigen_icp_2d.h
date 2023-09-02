#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"

using Vector2f = Eigen::Vector2f;

class ICP {
protected:
  struct PointPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointPair(const Vector2f& fixed_, const Vector2f& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointPair(){}
    Vector2f _fixed;
    Vector2f _moving;
  };
  using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair>>;

public:
  using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;

  ICP(const ContainerType& fixed_,
      const ContainerType& moving_,
      int min_points_in_leaf);

  void computeCorrespondences();
  
  void optimizeCorrespondences();

  void run(int max_iterations, Eigen::Isometry2f iso);

  const Eigen::Isometry2f& X() const {return _X;}
  Eigen::Isometry2f& X()  {return _X;}
  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
  inline const Eigen::Matrix<float, 3,1>& dx() const {return _dx;}
 
protected:
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  ContainerType _fixed;
  const ContainerType& _moving;
  Eigen::Isometry2f _X=Eigen::Isometry2f::Identity();
  TreeNodeType _kd_tree;
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  PointPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 3,1> _dx;
};

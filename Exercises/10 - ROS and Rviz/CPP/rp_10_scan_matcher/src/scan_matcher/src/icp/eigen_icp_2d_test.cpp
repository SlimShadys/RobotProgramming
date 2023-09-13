#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;


using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;


struct IncrementalScanMatcher {
  IncrementalScanMatcher() {}
  
  void compute(const Vector2fVector& current,
               const Eigen::Isometry2f& initial_guess=Eigen::Isometry2f::Identity()) {
    if (_previous.empty()) {
      _previous=current;
      return;
    }
    ICP icp(_previous, current, _min_points_in_leaf);
    icp.X()=initial_guess;
    icp.run(_num_iterations);
    _incremental_estimate=icp.X();
    _cumulative_estimate = _cumulative_estimate *_incremental_estimate;
    _previous=current;
  }

  int _num_iterations  = 10;
  int _min_points_in_leaf = 10;
  Vector2fVector _previous;
  Eigen::Isometry2f _incremental_estimate=Eigen::Isometry2f::Identity();
  Eigen::Isometry2f _cumulative_estimate=Eigen::Isometry2f::Identity();
};

IncrementalScanMatcher ism;

int main(int argc, char** argv) {
  using ContainerType=ICP::ContainerType;
  
  if (argc<5) {
    cerr << "usage: " << environ[0] << " tx ty az num_points";
    return -1;
  }
  float tx, ty, az;
  tx=atof(argv[1]);
  ty=atof(argv[2]);
  az=atof(argv[3]);
  cerr << "translation: " << tx << " " << ty << endl;
  cerr << "rotation: "    << az << endl;

  int num_points=atoi(argv[4]);
  
  // generate 1000 random points
  ContainerType fixed_points(num_points);
  for (auto& v: fixed_points) {
    v=Vector2f::Random()*100;
  }

  Eigen::Isometry2f motion;
  motion.linear()=Rtheta(az);
  motion.translation()=Vector2f(tx, ty);

  Eigen::Isometry2f moving_pose=Eigen::Isometry2f::Identity();
  Eigen::Isometry2f moving_estimate=Eigen::Isometry2f::Identity();

  ContainerType moving_points = fixed_points;
  ism.compute(moving_points);
  
  while (1) {
    moving_pose= moving_pose * motion;

    // generate vitual scan, seen from moving_pose;
    for (size_t pt_num = 0; pt_num<moving_points.size(); ++pt_num){
      moving_points[pt_num] = moving_pose.inverse() * fixed_points[pt_num];
    }
    
    // ICP icp(fixed_points, moving_points, 10);
    // icp.X()=moving_estimate;
    // icp.run(10);
    // moving_estimate=icp.X();

    ism.compute(moving_points);
    cerr << "gt: "  << endl;
    cerr << moving_pose.matrix() << endl;
    cerr << "estimate: "  << endl;
    cerr << ism._cumulative_estimate.matrix() << endl;
    char c;
    cin >>  c;
  }
}




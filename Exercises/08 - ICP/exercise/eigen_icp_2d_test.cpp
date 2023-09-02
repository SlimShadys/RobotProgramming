#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>
#include "eigen_icp_2d.h"

using namespace std;
extern const char ** environ;

using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

int main(int argc, char** argv) {
  using ContainerType=ICP::ContainerType;
  
  if (argc<6) {
    cerr << "usage: " << environ[0] << " tx ty az num_points max_iter";
    return -1;
  }
  float tx = atof(argv[1]);
  float ty = atof(argv[2]);

  float az = atof(argv[3]);

  int num_points = atoi(argv[4]);

  int max_iter = atoi(argv[5]);

  cout << "Running the script with the following args:" << endl;
  cout << "- Translation: " << tx << ", " << ty << endl;
  cout << "- Rotation: " << az << endl;
  cout << "- Num points: " << num_points << endl;
  cout << "- Max iterations: " << max_iter << endl;

  // generate 1000 random points
  ContainerType kd_points(num_points);
  for (auto& v: kd_points) {
    v=Vector2f::Random()*100;
  }

  Eigen::Isometry2f iso;
  iso.linear()=Rtheta(az);
  iso.translation()=Vector2f(tx, ty);

  ContainerType transformed_points=kd_points;
  for (auto& v: kd_points){
    v=iso*v;
  }
  ICP icp(kd_points, transformed_points, 10);
  icp.run(max_iter, iso);
}




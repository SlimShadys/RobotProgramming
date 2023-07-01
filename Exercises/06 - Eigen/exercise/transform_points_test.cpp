#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include "rotations.h"
#include "Eigen/Geometry"
#include "transform_points.h"
using namespace std;

int main(int argc, char** argv) {
  if (argc<2)
    return -1;
  // we open a file stream;

  std::ifstream is(argv[1]);
  Vector3fVector points;
  int num_points = loadPoints(points, is);
  cerr << "I read " << num_points << " from the stream " << endl;

  Vector3f angles;
  angles << 0.1, 0.2, 0.3;

  Eigen::Isometry3f iso;
  iso.linear() = Rxyz(angles);
  iso.translation().setZero();
  cerr << iso.matrix() << endl;
  transformInPlace(iso, points.begin(), points.end());
  
  savePoints(std::cout, points);

  

  /* TODO for you 

     1. change the above program to read n points in 2D
     2. change the program to read n points in 2D and 3D from a std::list

   */
  return 0;
}

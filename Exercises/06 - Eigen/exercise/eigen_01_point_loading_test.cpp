#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

using Vector2f = Eigen::Matrix<float, 2,1>;
using Vector2fList = std::list<Vector2f, Eigen::aligned_allocator<Vector2f> >;

struct StuffWithEigen {
  char c;
  Vector3f point1;
  Vector3f point2;
};

using StuffWithEigenVector = std::vector<StuffWithEigen>;



int main(int argc, char** argv) {
  cerr<< "the size is: " << sizeof(StuffWithEigen) << endl;
  
  StuffWithEigenVector my_container;
  for (int i=0; i<100; ++i) {
    my_container.push_back(StuffWithEigen());
  }

  for (const auto& item: my_container) {
    cout << item.point1;
    cout << item.point2;
    cout << (item.point1+item.point2);
    cout << endl;
  }
  
  if (argc<2)
    return -1;
  // we open a file stream;

  
  std::ifstream is(argv[1]);
  Vector2fList points;
  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;
  savePoints(std::cout, points);

  
  // std::ifstream is(argv[1]);
  // Vector3fVector points;
  // int num_points = loadPoints(points, is);
  // cerr << "I read" << num_points << "from the stream " << endl;
  // savePoints(std::cerr, points);


  /* TODO for you 

     1. change the above program to read n points in 2D
     2. change the program to read n points in 2D and 3D from a std::list

   */
  return 0;
}


#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

using Matrix2d = Eigen::Matrix<float, 2,2>;
using Vector2f = Eigen::Matrix<float, 2,1>;
using Vector2fList = std::list<Vector2f, Eigen::aligned_allocator<Vector2f> >;
using Vector3fList = std::list<Vector3f, Eigen::aligned_allocator<Vector3f> >;

struct StuffWithEigen {
  char c;
  Vector3f point1;
  Vector3f point2;
};

using StuffWithEigenVector = std::vector<StuffWithEigen>;

int main(int argc, char** argv) {
 
  // Just showing how push_back works with containers
  StuffWithEigenVector my_container;
  for (int i=0; i<100; ++i) {
    my_container.push_back(StuffWithEigen());
  }

  if (argc<2) return -1;

  int num_points2D = 0;
  int num_points3D = 0;
  Vector2fList points2D;
  Vector3fList points3D;
  std::ifstream is;

  bool run = true;
  while (1) {
    cerr << "Sorting: 1 = 2D points; 2 = 3D points" << endl;
    cerr << "Comparison formula: (p-m)^T * n < 0" << endl;
    cerr << "-------------------------" << endl;
    char s;
    cin >> s;
    switch (s) {
      case '1':
        cerr << "Sorting 2D points" << endl;
        //is.open("test_data/points_3d.dat");
        is.open(argv[1]);
        num_points2D = loadPoints(points2D, is, 2);
        cerr << "I read " << num_points2D << " points from the stream " << endl;
        savePoints(std::cout, points2D);

        // Sorting the points2D list based on the comparison formula
        points2D.sort(compareVectors2D);

        std::cerr << "-------------------------" << std::endl;
        std::cerr << "------ After sort -------" << std::endl;

        using PointType = typename Vector2fList::value_type;
        for (const auto& v: points2D) {
          for (int i=0; i<PointType::RowsAtCompileTime; ++i) {
            std::cout << "v(" <<i<< "): " << v(i) << " ";
          }
          std::cout << std::endl;
        }
        std::cerr << "-------------------------" << std::endl;
        is.close();
        break;
      case '2':
        cerr << "Sorting 3D points" << endl;
        //is.open("test_data/points_3d.dat");
        is.open(argv[1]);
        num_points3D = loadPoints(points3D, is, 3);
        cerr << "I read " << num_points3D << " points from the stream " << endl;
        savePoints(std::cerr, points3D);

        // Sorting the points3D list based on the comparison formula
        points3D.sort(compareVectors3D);

        std::cerr << "-------------------------" << std::endl;
        std::cerr << "------ After sort -------" << std::endl;

        using PointType3 = typename Vector3fList::value_type;
        for (const auto& v: points3D) {
          for (int i=0; i<PointType3::RowsAtCompileTime; ++i) {
            std::cout << "v(" <<i<< "): " << v(i) << " ";
          }
          std::cout << std::endl;
        }
        std::cerr << "-------------------------" << std::endl;
        is.close();
        break;
      default:
        cout << "Invalid option, exiting" << endl;
        run = false;
        break;
    }
    if (!run) break;
  }

  return 0;
}

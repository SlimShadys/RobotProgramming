#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include "rotations.h"
#include "Eigen/Geometry"

using namespace std;

int main(int argc, char** argv) {
  if (argc<2) return -1;

  ifstream is(argv[1]);
  //ifstream is("test_data/points_3d.dat");
  Vector3fVector points;
  int dimension = 3;
  int num_points = loadPoints(points, is, dimension);
  cerr << "I read " << num_points * dimension << " points from the stream " << endl;

  Vector3f angles;
  angles << 
        0.1, // alpha.x
        0.2, // alpha.y
        0.3; // alpha.z

  Eigen::Isometry3f iso;
  iso.linear() = Rxyz(angles); // Rotation matrix
  iso.translation().setZero(); // Translation
  cerr << "\niso.matrix(): " << endl;
  cerr << iso.matrix() << endl;
 
  // Apply the transformation to each point in the Vector3fVector
  for (auto &point : points) {
      // Apply rotation
      point = iso.linear() * point;

      // Apply translation
      point += iso.translation();
  }

  bool run = true;
  while (1) {
    cerr << "-------------------------" << endl;
    cerr << "Sorting: 1=x, 2=y, 3=z" << endl;
    cerr << "-------------------------" << endl;
    char s;
    cin >> s;
    switch (s) {
      case '1':
        cerr << "Sorting by x" << endl;
        sort(points.begin(), points.end(), [&](const Vector3f &a, const Vector3f &b) -> bool {
          return a[0] < b[0];
        });
        savePoints(cout, points);
        break;
      case '2':
        cerr << "Sorting by y" << endl;
        sort(points.begin(), points.end(), [&](const Vector3f &a, const Vector3f &b) -> bool {
          return a[1] < b[1];
        });        
        savePoints(cout, points);
        break;
      case '3':
        cerr << "Sorting by z" << endl;
        sort(points.begin(), points.end(), [&](const Vector3f &a, const Vector3f &b) -> bool {
          return a[2] < b[2];
        });        
        savePoints(cout, points);
        break;
      default:
        cout << "Invalid option, exiting" << endl;
        run = false;
    }
    if (!run) break;
    cerr << "-------------------------" << endl;
  }

  return 0;
}

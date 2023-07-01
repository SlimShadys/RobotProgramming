#include <algorithm>
#include <cassert>
#include <fstream>
#include <vector>

#include "point_with_index.h"

using namespace std;
using namespace rp;

using PointWithIndexVector = std::vector<PointWithIndex>;

struct CompareIndex {
  bool operator()(const PointWithIndex& a, const PointWithIndex& b) const {
    return a.index < b.index;
  }
};

struct CompareCoordinate {
  const int coordinate;
  CompareCoordinate(int coord) : coordinate(coord) {}
  bool operator()(const PointWithIndex& a, const PointWithIndex& b) const {
    switch (coordinate) {
      case 0: // Sorting by X
        return a.value.at(0) < b.value.at(0);
      case 1: // Sorting by Y
        return a.value.at(1) < b.value.at(1);
      case 2: // Sorting by Z
        return a.value.at(2) < b.value.at(2);
      default:
        return false;
    }
  }
};

int main(int argc, char** argv) {
  //assert(argc > 1);
  //ifstream is(argv[1]);
  ifstream is("../test_data/point_multisort_data.txt");
  PointWithIndexVector points;
  while (is) {
    PointWithIndex p;
    is >> p;
    if (is) {
      points.push_back(p);
    }
  }
  cerr << "loaded " << points.size() << " points from file [" << argv[1] << "]" << endl;

  bool run = true;
  while (1) {
    cerr << "Sorting: 1=index, 2=x, 3=y, 4=z" << endl;
    cerr << "-------------------------" << endl;
    char s;
    cin >> s;
    switch (s) {
      case '1':
        cerr << "Sorting by index" << endl;
        std::sort(points.begin(), points.end(), CompareIndex());
        break;
      case '2':
        cerr << "Sorting by x" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(0));
        break;
      case '3':
        cerr << "Sorting by y" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(1));
        break;
      case '4':
        cerr << "Sorting by z" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(2));
        break;
      default:
        cout << "Invalid option, exiting" << endl;
        run = false;
    }
    if (!run) break;
    for (const auto& p : points) {
      cerr << p << endl;
    }
    cerr << "-------------------------" << endl;
  }
}

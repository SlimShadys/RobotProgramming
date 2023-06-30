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
    // Todo: this compares the indices
    // what should I return
    return true;
  }
};

struct CompareCoordinate {
  const int coordinate;
  CompareCoordinate(int coord) : coordinate(coord) {}
  bool operator()(const PointWithIndex& a, const PointWithIndex& b) const {
    // Todo: this compares the coordinate index
    // passed on construction
    // what should I return?
    return true;
  }
};

int main(int argc, char** argv) {
  assert(argc > 1);
  ifstream is(argv[1]);
  PointWithIndexVector points;
  while (is) {
    PointWithIndex p;
    is >> p;
    if (is) {
      points.push_back(p);
    }
  }
  cerr << "loaded " << points.size() << " points from file [" << argv[1] << "]"
       << endl;

  bool run = true;
  while (1) {
    cerr << "Sorting: 1=index, 2=x, 3=y,4=z" << endl;
    char s;
    cin >> s;
    switch (s) {
      case '1':
        cerr << "sorting by index" << endl;
        std::sort(points.begin(), points.end(), CompareIndex());
        break;
      case '2':
        cerr << "sorting by x" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(0));
        break;
      case '3':
        cerr << "sorting by y" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(1));
        break;
      case '4':
        cerr << "sorting by z" << endl;
        std::sort(points.begin(), points.end(), CompareCoordinate(2));
        break;
      default:
        cout << "invalid option, exiting" << endl;
        run = false;
    }
    if (!run) break;
    for (const auto& p : points) {
      cerr << p << endl;
    }
  }
}

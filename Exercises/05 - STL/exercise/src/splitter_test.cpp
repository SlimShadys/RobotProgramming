#include "splitter.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <list>

#include "point_with_index.h"

using namespace rp;
using namespace std;

using PointWithIndexList = std::list<PointWithIndex>;

struct OddFirst {
  bool operator()(const PointWithIndex& p) { return p.index % 2; }
};

int main(int argc, char** argv) {
  assert(argc > 1);
  ifstream is(argv[1]);
  PointWithIndexList points;
  while (is) {
    PointWithIndex p;
    is >> p;
    if (is) {
      points.push_back(p);
    }
  }
  cerr << "loaded " << points.size() << " points from file [" << argv[1] << "]"
       << endl;
  split(points.begin(), points.end(), OddFirst());

  for (const auto& p : points) std::cerr << p << std::endl;
}

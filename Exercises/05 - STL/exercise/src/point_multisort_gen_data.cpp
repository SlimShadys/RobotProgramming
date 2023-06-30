#include "point_with_index.h"
#include <cassert>

using namespace std;
using namespace rp;


int main(int argc, char** argv) {
  assert(argc<2);
  int n_points=atoi(argv[1]);
  cerr << "requested to generate " << n_points << " random points on stdout" << endl;
  for (int i=0; i<n_points; ++i) {
    PointWithIndex p;
    p.index=i;
    randFill(p.value);
    cout << p << endl;
  }
}
  

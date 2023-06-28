#include "simple_geometry.h"
using namespace std;

ostream& operator << (ostream& os, const Point& p) {
  os << p.x << " " << p.y;
  return os;
}


ostream& operator << (ostream& os, const IntPoint& p) {
  os << p.x << " " << p.y;
  return os;
}

ostream& operator << (ostream& os, const Pose& pose) {
  os << pose.translation << " " << pose.theta;
  return os;
}

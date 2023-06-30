#pragma once
#include "static_vec.h"
#include <iostream>

namespace rp {
  using namespace std;
  
  using Vec3f=Vec_<float, 3>;

  struct PointWithIndex {
    int index;
    Vec3f value;
  };

  istream& operator>>(istream& is, PointWithIndex& p){
    is >> p.index >> p.value.at(0) >> p.value.at(1) >> p.value.at(2);
    return is;
  }

  ostream& operator<<(ostream& os, const PointWithIndex& p){
    os << p.index << " "
       << p.value.at(0) << " "
       << p.value.at(1) << " "
       << p.value.at(2);
    return os;
  }
}

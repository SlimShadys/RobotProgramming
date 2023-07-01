#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include "eigen_00_basics.h"
#include <iostream>
#include <fstream>

// we define a std vector of 3d points, the allocator helps
// the optimizer for aligned accesses. Required with -O3 (otherwise the program crashes)
using Vector3fVector=std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;

// we define a std vector of 2d points
using Vector2fVector=std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;

// we write a generic point loader
// template magic.
// the Vector::value_type, is the template type of the vector element
// the Eigen::Matrix<...>::RowsAtCompileTime is a constant telling
// how many rows the matrix has

template <typename ContainerType_>
int loadPoints(ContainerType_& dest, std::istream& is) {
  using PointType = typename ContainerType_::value_type;
  
  constexpr int dim=PointType::RowsAtCompileTime;
  while (is.good()) {
    PointType v;
    // we read one element at t time, and we put it in v(i)
    for (int i=0; i<dim; ++i) {
      is >> v(i);
    }
    if(! is.good())
      break;
    // at the end we push back the element in the container
    dest.push_back(v);
  }
  return dest.size();
}

template <typename ContainerType_>
int savePoints(std::ostream& os, const ContainerType_& src) {
  using PointType= typename ContainerType_::value_type;
  int dim=PointType::RowsAtCompileTime;

  // we use the ":" to iterate in a container
  for (const auto& v: src) {
    for (int i=0; i<dim; ++i) {
      os << v(i) << " ";
    }
    os << std::endl;
  }
  return src.size();
}

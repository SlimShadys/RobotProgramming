#pragma once
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include "eigen_00_basics.h"
#include <iostream>
#include <fstream>
#include <random>

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
int loadPoints(ContainerType_& dest, std::istream& is, int dimension) {
    using PointType = typename ContainerType_::value_type;

    if (dimension <= 0 || dimension > PointType::RowsAtCompileTime) {
        std::cerr << "Invalid dimension specified!" << std::endl;
        return 0;
    }

    while (is.good()) {
        PointType v;

        for (int i = 0; i < dimension; ++i) {
            is >> v(i);
        }

        if (!is.good())
            break;

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
      std::cout << "v(" <<i<< "): " << v(i) << " ";
    }
    std::cout << std::endl;
  }
  return src.size();
}

float getRandomFloat(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

// Function to generate a random Vector2f with values between min and max for each element
Vector2f getRandomVector2f(float min, float max) {
    Vector2f randomVec;
    randomVec << getRandomFloat(min, max), getRandomFloat(min, max);
    return randomVec;
}

// Function to generate a random Vector2f with values between min and max for each element
Vector3f getRandomVector3f(float min, float max) {
    Vector3f randomVec;
    randomVec << getRandomFloat(min, max), getRandomFloat(min, max), getRandomFloat(min, max);
    return randomVec;
}

bool compareVectors2D(const Vector2f& m, const Vector2f& n) {
   // Generate p only once. Static as requested.
   // We generate with numbers between -2 and 2.
    static Vector2f p = getRandomVector2f(-2.0f, 2.0f);
    return (p - m).transpose() * n < 0;
}

bool compareVectors3D(const Vector3f& m, const Vector3f& n) {
   // Generate p only once. Static as requested.
   // We generate with numbers between -2 and 2.
    static Vector3f p = getRandomVector3f(-2.0f, 2.0f);
    return (p - m).transpose() * n < 0;
}
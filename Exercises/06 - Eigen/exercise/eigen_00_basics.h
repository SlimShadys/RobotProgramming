#pragma once

// eigen is a header only library for linear algebra.
// it implements many of the functions commonly used on vectors and matrices
// through the template mechanism it allows for parametric scalar type (int, double, float, whatever you like)
// It supports both fixed size (effiicent) objects and variable size objects.
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

// using acts as a typedef, and can be used to ease the template names
// using <new name> = <type>

// we define a Vector3f object as a matrix bof float having fixed size 3x1
using Vector3f = Eigen::Matrix<float,3,1>;

// we define a Vector2f object as a matrix bof float having fixed size 2x1
using Vector2f = Eigen::Matrix<float,2,1>;

// we define a Matrix3f object as a matrix bof float having fixed size 3x3
using Matrix3f = Eigen::Matrix<float,3,3>;

// we define a Matrix2f object as a matrix bof float having fixed size 2x2
using Matrix2f = Eigen::Matrix<float,2,2>;


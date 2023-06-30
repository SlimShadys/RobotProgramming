#pragma once
#include "dynamic_vec.h"
// untested

namespace rp {
  class FloatMat {
  public:
    FloatMat(int rows, int cols);
    FloatMat(const FloatMat& other);
    ~FloatMat();
    FloatMat& operator=(const FloatMat& other);
    FloatMat& operator+=(const FloatMat& other);
    FloatMat& operator-=(const FloatMat& other);
    FloatMat& operator*=(float& scalar);
    FloatVec operator*(FloatVec& v) const;
    FloatMat transpose() const;
    FloatMat& transposeInPlace();
    FloatMat operator+(const FloatMay& other) const;
    FloatMat operator-(const FloatMat& other) const;
    FloatMat operator*(const FloatMat& other) const;
    const int dimension() const;
    const int rows() const;
    const int cols() const;
    float& at(int pos);
    const float& at(int pos);
    float& at(int r, int c);
    const float& at(int r, int c);
  protected:
    float* _values;
    int _dimension;
    int rows, int cols;
  };
  
    
}

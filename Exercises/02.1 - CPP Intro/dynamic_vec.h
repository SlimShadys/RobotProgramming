#pragma once
// untested

namespace rp {
  class FloatVec {
  public:
    FloatVec(int size);
    FloatVec(const FloatVec& other);
    ~FloatVec();
    FloatVec& operator=(const FloatVec& other);
    FloatVec& operator+=(const FloatVec& other);
    FloatVec& operator-=(const FloatVec& other);
    FloatVec& operator*=(float& scalar);
    float dot(const FloatVec& other) const;
    float squaredNorm(const FloatVec& other) const;
    float norm(const FloatVec& other) const;
    FloatVec operator+(const FloatVec& other) const;
    FloatVec operator-(const FloatVec& other) const;
    const int dimension() const;
    float& at(int pos);
    const float& at(int pos);
  protected:
    float* _values;
    int _dimension;
  };
  
    
}

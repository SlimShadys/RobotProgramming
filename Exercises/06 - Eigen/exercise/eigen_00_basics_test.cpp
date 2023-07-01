
// eigen is a header only library for linear algebra.
// it implements many of the functions commonly used on vectors and matrices
// through the template mechanism it allows for parametric scalar type (int, double, float, whatever you like)
// It supports both fixed size (effiicent) objects and variable size objects.
#include <Eigen/Core>
#include <iostream>
#include "eigen_00_basics.h"

using namespace std;

using Matrix3f = Eigen::Matrix<float, 3, 3>;
using Matrix2f = Eigen::Matrix<float, 2, 2>;
using Matrix2_3f = Eigen::Matrix<float, 2, 3>;
using Vector3f   = Eigen::Matrix<float, 3,1>;

int main() {
  Eigen::Matrix<float, 3,3> m1;
  Matrix3f m2;
  m2.setZero();
  
  m1 << 1,2,3,
        4,5,6,
        7,8,9;

  
  std::cerr << "m1: " << endl << m1 << endl;
  std::cerr << "m2: " << endl << m2 << endl;
  m2=m1; // we can assign;

  std::cerr << "m2: " << endl<< m2 << endl;
  Matrix3f m3 = m2 * m1;

  std::cerr << "m3: " << endl << m3 << endl;

  Vector3f v=m3.col(0);  // assign first column of m3 to v;

  std::cerr << "v: " << endl << v << endl;
  
  std::cerr << "m3*v: " << endl << m3*v << endl;

  Matrix2_3f m4;
  m4.setZero();
  m4(0,0)=1;
  m4(1,1)=2;
  m4(0,2)=0.5;
  m4(1,2)=0.5;

  
  // what happens if we uncomment this?
  m4(1,3)=0.5;

  std::cerr << "m4: " << endl << m4 << endl;

  Eigen::Matrix<float, 2,2> m44=m4*m4.transpose();
  std::cerr << "m44 " << endl << m44<< endl;

  // the keyword auto tries to infer the type from the result of the
  // expression, that's complicated :)
  auto m55=m2*(m4.transpose()*m4)+0.5*m1;
  std::cerr << "m55 " << endl << m55<< endl;
}

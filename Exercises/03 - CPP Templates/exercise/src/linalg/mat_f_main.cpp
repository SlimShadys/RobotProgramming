#include <iostream>
#include "mat_f.h"
using namespace std;

int main()
{
  VecF v1(3);
  v1.at(0) = 1;
  v1.at(1) = 2;
  v1.at(2) = 3;
  cout << "Vector V1: " << v1 << endl;

  MatF m(3, 3);
  m.fill(0.);
  cout << "M matrix: " << m << endl;
  
  MatF m2(3, 3);
  m2.fill(0);
  m2.at(0, 0) = 1;
  m2.at(1, 1) = 2;
  m2.at(2, 2) = 3;
  cout << "M2 matrix: " << m2 << endl;

  MatF m3 = m + m2;
  cout << "M3 matrix: " << m3 << endl;
  cerr << "-------------------------" << endl;

  cout << "M + M2: " << m + m2 << endl;
  cout << "M3 - M2 : " << m3 - m2 << endl;
  cout << "M * V1  : " << m * v1 << endl;
  cout << "M3 * V1 : " << m3 * v1 << endl;
  cerr << "-------------------------" << endl;

  MatF m4(3, 4);
  cerr << "Dimension of M4 Matrix: " << m4.dimension << endl;
  m4.fill(1);
  cout << "M4 matrix: " << m4 << endl;

  cout << "(M3 * M4)^T: " << (m3 * m4).transpose() << endl;
  cerr << "-------------------------" << endl;

  MatF m5(3, 4);
  m5.randFill();
  cout << "M5 matrix: " << m5 << endl;

  cout << "M5 * M5^T: " << m5 * m5.transpose() << endl;
  cerr << "-------------------------" << endl;

  m.randFill();
  cout << "M after random filling: " << m << endl;
  cerr << "-------------------------" << endl;

  cout << "M - M5 * M5^T" << m - m5 * m5 << endl;
  cerr << "-------------------------" << endl;

  cout << "DONE" << endl;
}

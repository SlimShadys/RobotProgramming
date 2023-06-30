#include <iostream>
#include "vec_f.h"

using namespace std;

int main() {
  VecF v1(3);
  for (int i=0; i<v1.dim; ++i)
    v1.at(i)=i;

  cerr << "v1: " << v1 << endl;

  VecF v2(v1);
  cerr << "v2: " << v2 << endl;

  VecF v3 = v2;
  cerr << "v3: " << v3 << endl;

  cerr << "V1 + V2: " << v1+v2 << endl;
  cerr << "V1 - V2: " << v1-v2 << endl;

  cerr << "-------------------------" << endl;
  v1.at(2)+=4;
  cerr << "Added 4 to v1[2]. Now v1 is: " << v1 << endl;
  cerr << "-------------------------" << endl;

  cerr << "V1 + (V2*2.f): " << v1+(v2*2.f) << endl;
  cerr << "-------------------------" << endl;

  cerr << "V1 * V2: " << v1 * v2 << endl;
  cerr << "-------------------------" << endl;
}

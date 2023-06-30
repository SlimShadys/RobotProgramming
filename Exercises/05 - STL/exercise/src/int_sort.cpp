#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

using IntVector = std::vector<int>;

int main(int argc, char** argv) {
  assert(argc > 1);
  ifstream is(argv[1]);
  IntVector values;
  while (is) {
    // TODO: load the values
  }
  std::sort(values.begin(), values.end());
  // TODO: print the values
}

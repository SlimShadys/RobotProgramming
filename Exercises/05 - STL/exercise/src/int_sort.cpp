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
    int num = 0;
    is >> num;
    values.push_back(num);
  }

  std::sort(values.begin(), values.end()); // Standard sort

  // Print values
  for (auto val: values) {
    cout << "Val: " << val << endl;
  }
}

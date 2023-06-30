#include <iostream>
#include "btree.h"

using namespace std;

int main() {
  TreeNodeInt* n=new TreeNodeInt(5);
  n->add(10);
  n->add(1);
  n->add(2);
  n->add(4);
  cerr << "Values successfully added. Printing the order now:" << endl;
  
  n->print(std::cout);

  cerr << "Address of obj 4: " << n->find(4) << endl;
  
  delete n;
  return 0;
}

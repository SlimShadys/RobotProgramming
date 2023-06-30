#include "binary_tree_template.h"
#include <iostream>

using TreeNodeInt=TreeNode_<int>;

int main() {

  TreeNodeInt::PtrType  root(new TreeNodeInt(0));
  root->insert(10);
  root->insert(-10);
  root->insert(5);
  root->insert(15); 
  root->insert(-5);
  root->insert(-15);
  root->print(std::cout);
}

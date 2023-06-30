#include "btree.h"
#include <stdexcept>

using namespace std;

TreeNodeInt::TreeNodeInt(int value_,
                         TreeNodeInt* left_,
                         TreeNodeInt* right_):
  // Equal to Python way:
  // - self.value = value_
  // - self.left = left_
  // - self.right = right_
  value(value_),
  left(left_),
  right(right_){}

TreeNodeInt::~TreeNodeInt(){
  if (left)
    delete left;
  if (right)
    delete right;
}

TreeNodeInt* TreeNodeInt::find(int value_) {
  if (value==value_) // We found the value
    return this;
  else if (value_<value) {
    // if there is a left child, continue the search in the left, otherwise return null
    if(! left) {
      return 0;
    } else {
      return left->find(value_);
    }
  }
  else if (value_>value) {
    // if there is a right child, continue the search in the right, otherwise return null
    if (! right) {
      return 0;
    } else {
      return right->find(value_);
    }
  } else {
    throw std::invalid_argument("No value to find. There might be an error.");
  }
}
  
TreeNodeInt* TreeNodeInt::add(int value_) {
  // If we already have this value, don't add it
  if (value==value_) {
    cout << "Value " << value_ << " already present!" << endl;
    return 0;
  }
  else if (value_<value) { // If the value we want to add is lower than the actual value, we go left
    if (! left) { // If there is no left node active, we add a left node with this value
      left=new TreeNodeInt(value_);
      return left;
    }
    return left->add(value_); // Otherwise we recursively search for a new left-spot.
  }
  else if (value_>value) {
    if (! right) { // If there is no right node active, we add a right node with this value
      right=new TreeNodeInt(value_);
      return right;
    }
    return right->add(value_); // Otherwise we recursively search for a new right-spot.
  }
  else {
    throw std::invalid_argument("No value to add. There might be an error.");
  }
}

void TreeNodeInt::print(std::ostream& os) {
  if (left)
    left->print(os);
  os << value << " " << endl;
  if (right)
    right->print(os);
}

std::ostream& operator << (std::ostream& os, const TreeNodeInt& node) {
  if (node.left)
    os << *(node.left);
  os << node.value << " ";
  if (node.right)
    os << *(node.right);
  return os;
}

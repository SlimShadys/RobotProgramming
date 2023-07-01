#pragma once
#include "default_compare.h"
#include <iostream>
#include <memory>

template <typename T_,
          typename Compare_=DefaultCompare_<T_> >

struct TreeNode_{
  using ValueType=T_;
  using ThisType=TreeNode_<ValueType, Compare_>;
  static const Compare_ _compare;
  ValueType _value;
  using PtrType = std::shared_ptr<ThisType>;
  PtrType _left, _right;

  TreeNode_(const ValueType& value_,
            PtrType left_=0,
            PtrType right_=0):
    _value(value_),
    _left(left_),
    _right(right_)
  {}

  PtrType insert(const ValueType& value_) {
    if (_compare(value_, _value)) {
      if (! _left) {
        _left = PtrType(new TreeNode_(value_, _left=0));
        return _left;
      }
      return _left->insert(value_);
    }
    if (_compare(_value, value_)) {
      if (! _right) {
        _right = PtrType(new TreeNode_(value_, _right=0));
        return _right;
      }
      return _right->insert(value_);
    }
    return nullptr;
  }

  void print(std::ostream& os) {
    if (_left)
      _left->print(os);
    os << _value << "\n";
    if (_right)
      _right->print(os);
  }
};

// out of class static member declaration
template <typename T_, typename Compare_>
const Compare_ TreeNode_<T_,Compare_>::_compare;
//    ^^^^^^^^ ^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^
//    type     class                   member

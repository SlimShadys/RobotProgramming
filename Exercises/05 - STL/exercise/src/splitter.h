#pragma once
#include <algorithm>

template <typename IteratorType_, typename PredicateType_>
IteratorType_ split(IteratorType_ begin, IteratorType_ end,
                    PredicateType_ predicate) {
  // this represents the type of the element contained
  using ValueType = typename IteratorType_::value_type;
  // this represents the type of the iterator
  using IteratorType = IteratorType_;

  // our game is the following
  // we have an iterator in front, which we move forward
  // let's call it "lower"
  auto lower = begin;

  // and one iterator on the back, which we move backwards
  // since not all STL does iterators can't be moved backwards
  // we can construct (see documentation)
  // a reverse_iterator, that when incremented moves backwards
  auto upper = std::make_reverse_iterator(end);

  // reverse and direct iterators cannot be compared straight away,
  // To do so you need to extract a normal iterator
  // from a reverse one using base()

  // our algorithm stops when lower and upper are the same
  // at each step, if the predicate on the element pointed by upper
  // is true, we increment upper
  // otherwise we swap the values pointed by upper and lower
  // and we increment lower

  while (lower != upper.base()) {
    ValueType& v_lower = *lower;
    ValueType& v_upper = *upper;
    if (predicate(v_lower)) {
      // todo anything to write here?
    } else {
      // todo: anything to write here?
      // Tip: incrementing a reverse iterator goes backwards
    }
  }
  return upper.base();
}

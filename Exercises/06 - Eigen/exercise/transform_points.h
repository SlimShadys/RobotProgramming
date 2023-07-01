#pragma once

template <typename TransformType,
          typename IteratorType>
void transformInPlace(const TransformType& t,
                      IteratorType begin,
                      IteratorType end){
  while (begin!=end) {
    *begin= (t * (*begin));
    ++begin;
  }
};

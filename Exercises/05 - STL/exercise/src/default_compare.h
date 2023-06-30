#pragma once

template <typename T>
struct DefaultCompare_{
  inline bool operator()(const T&a, const T&b) const {
    return a<b;
  }
};

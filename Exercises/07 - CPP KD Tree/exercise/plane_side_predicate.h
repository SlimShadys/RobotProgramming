#pragma once

template <typename PointType>
struct PlaneSidePredicate_ {
  PlaneSidePredicate_(const PointType&  center_,
                      const PointType& direction_):
    _center(center_),
    _direction(direction_){};
  const PointType& _center;
  const PointType& _direction;
  inline bool operator()(const PointType& v) const{
    return (v-_center).dot(_direction)<0;
  }
};

#pragma once
#include <cmath>
#include <iostream>

struct Point {
  float x,  y;
  Point(const float& x_=0, const float& y_=0):
    x(x_),
    y(y_){}
  
  inline Point operator+(const Point& other) const {
    return Point(x+other.x, y+other.y);
  }

  inline Point operator-(const Point& other) const {
    return Point(x-other.x, y-other.y);
  }

  inline Point operator*(const float& s) const {
    return Point(x*s, y*s);
  }

};

struct IntPoint {
  int x,  y;
  IntPoint(const int& x_=0, const int& y_=0):
    x(x_),
    y(y_){}
  
  inline IntPoint operator+(const IntPoint& other) const {
    return IntPoint(x+other.x, y+other.y);
  }

  inline IntPoint operator-(const IntPoint& other) const {
    return IntPoint(x-other.x, y-other.y);
  }

  inline IntPoint operator*(const int& s) const {
    return IntPoint(x*s, y*s);
  }

  inline int operator*(const IntPoint& other) const {
    return x*other.x + y*other.y;
  }

};

struct Pose {
  Point translation;
  float theta;

  Pose(const Point& translation_=Point(), const float& theta_=0):
    translation(translation_),
    theta(theta_){}

  Pose(const float& x_, const float& y_, const float& theta_=0):
    translation(x_,y_),
    theta(theta_){}


  
  inline Point operator*(const Point& p) const {
    float s=sin(theta), c=cos(theta);
    return translation + Point(c*p.x - s*p.y,
                                 s*p.x + c*p.y);
  }

  //(R1 | t1) (R2 | t2) = (R1*R2 | t1 + R1*t2)
  inline Pose operator* (const Pose& other) const {
    const Pose& my_ref(*this);
    
    return Pose (my_ref*other.translation, theta+other.theta); 
  }


};


std::ostream& operator << (std::ostream& os, const Point& p);


std::ostream& operator << (std::ostream& os, const IntPoint& p);

std::ostream& operator << (std::ostream& os, const Pose& pose);

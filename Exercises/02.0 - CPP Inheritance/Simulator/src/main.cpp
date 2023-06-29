#include <sys/time.h>

#include <cmath>
#include <iostream>

#include "lidar.h"
#include "pan_unit.h"
#include "robot.h"
#include "simple_geometry.h"
#include "world.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec * 1000 + tv.tv_usec * 1e-3;
}

int main(int argc, char** argv) {
  World w;
  //w.loadFromImage(argv[1]);
  // We load it manually for G++. Not needed when we use CMake
  w.loadFromImage("/home/lattinone/RobotProgramming/Exercises/02.0 - CPP Inheritance/Simulator/src/background.png");

  IntPoint middle(w.rows / 2, w.cols / 2);
  Pose robot_pose;
  robot_pose.translation = w.grid2world(middle);
  Robot r(0.3, &w, robot_pose);
  PanUnit pu(&r, 0.05, Pose(1.0, 0.0, 0.0));
  pu.setAngle(M_PI_2);
  Lidar l(M_PI, 10, 180, &pu, Pose(0.1, 0, 0));

  float delay = 0.1;
  int k;

  while (1) {
    w.timeTick(delay);
    w.draw(r.rv, r.tv);

    k=cv::waitKeyEx(delay*1000)&255;
    switch (k) {
      case 81:
        r.rv += 0.05;
        break;  // arrow left
      case 82:
        r.tv += 0.1;
        break;  // arrow up
      case 83:
        r.rv -= 0.05;
        break;  // arrow right
      case 84:
        r.tv -= 0.1;
        break;  // arrow dw
      case 32:
        r.tv = 0;
        r.rv = 0;
        break;  // spacebar
      case 27:
        return 0;  // space
      default:;
    }
    cerr << "Rotation velocity    : " << r.rv << endl;
    cerr << "Translation velocity : " << r.tv << endl;
    cerr << "-------------------------" << endl;
  }
}
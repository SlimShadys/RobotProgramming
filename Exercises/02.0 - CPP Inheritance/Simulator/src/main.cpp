#include <sys/time.h>

#include <cmath>
#include <iostream>

#include "lidar.h"
#include "pan_unit.h"
#include "robot.h"
#include "simple_geometry.h"
#include "world.h"

using namespace std;

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec * 1000 + tv.tv_usec * 1e-3;
}

int main(int argc, char** argv) {
  World w;
  w.loadFromImage(argv[1]);

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
    double t_start = timeMillisec();
    w.timeTick(delay);
    double t_end = timeMillisec();
    cerr << "duration" << t_end - t_start << endl;
    cerr << "image_size: " << w.rows << " " << w.cols << endl;
    w.draw();

    // k=cv::waitKeyEx(delay*1000)&255;
    k = cv::waitKeyEx(0) & 255;
    switch (k) {
      case 81:
        r.rv += 0.05;
        break;  // arow left
      case 82:
        r.tv += 0.1;
        break;  // arow up
      case 83:
        r.rv -= 0.05;
        break;  // arow right
      case 84:
        r.tv -= 0.1;
        break;  // arow dw
      case 32:
        r.tv = 0;
        r.rv = 0;
        break;  // spacebar
      case 27:
        return 0;  // space
      default:;
    }
    cerr << "k: " << (int)k << endl;
  }
}
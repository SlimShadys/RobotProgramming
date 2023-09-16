#include <sys/time.h>

#include <cmath>
#include <iostream>

#include "lidar.h"
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

  // Create a World object
  World w;
  w.loadFromImage("../test_data/background.png");

  // Create a shared pointer to the World
  std::shared_ptr<World> worldSharedPtr = std::make_shared<World>(w);

  //IntPoint middle(w.rows / 2, w.cols / 2);
  IntPoint startPointDIAG(30.9061 / w.res, 51.4195 / w.res);
  //IntPoint startPointMilitary((w.rows / 2 + 100), w.cols / 2);

  Pose robot_pose;
  robot_pose.translation = w.grid2world(startPointDIAG);
  
  /**
   * If we pass directly a Robot object inside make_shared, we are trying to create a shared pointer from an already constructed object,
   * which is not how std::make_shared is intended to be used.
   * 
   * In fact, we are directly creating a std::shared_ptr<Robot> object and 
   * costructing a Robot object directly inside the std::make_shared function.
  */
  std::shared_ptr<Robot> robotSharedPtr = std::make_shared<Robot>(0.3, worldSharedPtr, robot_pose);

  /**
   * TO USE, IN CASE | FOR NOW LET'S USE A NORMAL "SHAREDPTR" WITHOUT DYNAMIC CAST
   * 
   * Here, we're trying to cast robotSharedPtr (which points to a Robot object) into a std::shared_ptr<WorldItem> (which is a base class of Robot).
   * With this being said, robotPointer will point to the same object as robotSharedPtr, but with the type std::shared_ptr<WorldItem>.
   * 
   * This is crucial because we want to construct a pointer to the Robot object, such that when we instantiate a Lidar object, we pass this pointer
   * and the Lidar gets linked with the Robot.
  */
  //std::shared_ptr<WorldItem> robotPointer = std::dynamic_pointer_cast<WorldItem>(robotSharedPtr);

  Lidar l(M_PI, 10, 180, robotSharedPtr, Pose(0.4, 0, 0));
  //Lidar l2(M_PI, 5, 100, robotSharedPtr, Pose(-0.4, 0, M_PI));

  float delay = 0.04;
  int k;

  while (1) {

    worldSharedPtr->timeTick(delay);
    worldSharedPtr->draw(robotSharedPtr->rv, robotSharedPtr->tv);

    k=cv::waitKeyEx(delay*1000)&255;
    switch (k) {
      case 81:  // Left arrow
        robotSharedPtr->rv += 0.10;
        break;  
      case 82:  // Up arrow
        robotSharedPtr->tv += 0.2;
        break;  
      case 83:  // Right arrow
        robotSharedPtr->rv -= 0.10;
        break;  
      case 84:  // Down arrow
        robotSharedPtr->tv -= 0.2;
        break;  
      case 32:  // Spacebar
        robotSharedPtr->tv = 0;
        robotSharedPtr->rv = 0;
        break;  
      case 27:  // Esc
        return 0;
      default:
        robotSharedPtr->rv = 0;
        continue;
    }
    // cerr << "X : " << robotSharedPtr->pose.translation.x << endl;
    // cerr << "Y : " << robotSharedPtr->pose.translation.y  << endl;
    // cerr << "-------------------------" << endl;
    // cerr << "Rotation velocity    : " << r.rv << endl;
    // cerr << "Translation velocity : " << r.tv << endl;
    // cerr << "-------------------------" << endl;
  }
}
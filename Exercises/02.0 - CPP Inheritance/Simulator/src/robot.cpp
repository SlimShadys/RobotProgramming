#include "robot.h"
using namespace std;

RobotBase::RobotBase(float radius_, World* w, const Pose& pose_)
    : WorldItem(w, pose_) {
  radius = radius_;
}

RobotBase::RobotBase(float radius_, WorldItem* p_, const Pose& pose_)
    : WorldItem(p_, pose_) {
  radius = radius_;
}

void RobotBase::draw() {
  int int_radius = radius * world->inv_res;
  int r2 = int_radius * int_radius;
  cerr << "r2 = " << r2;
  IntPoint p = world->world2grid(poseInWorld().translation);
  cerr << "origin = " << p << endl;
  for (int r = -int_radius; r <= int_radius; ++r) {
    for (int c = -int_radius; c <= int_radius; ++c) {
      IntPoint off(r, c);
      if (off * off > r2) continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (world->inside(p_test)) {
        world->_display_image.at<uint8_t>(p_test.x, p_test.y) = 0;
      }
    }
  }
}

void Robot::timeTick(float dt) {
  Pose motion(tv * dt, 0, rv * dt);
  Pose next_pose = pose * motion;
  IntPoint ip = world->world2grid(next_pose.translation);

  int int_radius = radius * world->inv_res;
  if (!world->collides(ip, int_radius)) pose = next_pose;
}

void HolonomicRobot::timeTick(float dt) {
  // Create the delta transform 'motion' according to
  // the holonomic platform's kinematics
  /**
   * dx = tv_x * dt
   * dy = tv_y * dt
   * dr = rv * dt
   */
  Pose motion(0.0, 0.0, 0.0);  // <- TODO
  Pose next_pose = pose * motion;
  IntPoint ip = world->world2grid(next_pose.translation);

  int int_radius = radius * world->inv_res;
  if (!world->collides(ip, int_radius)) pose = next_pose;
}

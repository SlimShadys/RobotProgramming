#include "robot.h"
using namespace std;

Robot::Robot(float radius_, std::shared_ptr<World> w, const Pose& pose_)
    : WorldItem(w, pose_) {
  radius = radius_;
}

Robot::Robot(float radius_, std::shared_ptr<WorldItem> p_, const Pose& pose_)
    : WorldItem(p_, pose_) {
  radius = radius_;
}

void Robot::draw() {
  int int_radius = radius * world->inv_res;
  int r2 = int_radius * int_radius;
  IntPoint p = world->world2grid(poseInWorld().translation);
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

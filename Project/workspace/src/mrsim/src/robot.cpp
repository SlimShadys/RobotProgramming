#include "robot.h"
#include <iostream>
#include <string>

using namespace std;

Robot::Robot(float radius_, std::shared_ptr<World> w_, const Pose& pose_)
    : WorldItem(w_, pose_) {
  radius = radius_;
}

Robot::Robot(float radius_, std::shared_ptr<WorldItem> p_, const Pose& pose_)
    : WorldItem(p_, pose_) {
  radius = radius_;
}

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, std::shared_ptr<World> w_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(w_, pose_) {
    id = id_;
    type = type_;
    frame_id = frame_id;
    namespc = namespace_;
    radius = radius_;
    max_rv = max_rv_;
    max_tv = max_tv_;
    parent = parent_;
    isChild = false;
    w = w_;
    relativePose = pose_;
}

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, std::shared_ptr<WorldItem> p_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(p_, pose_) {
    id = id_;
    type = type_;
    frame_id = frame_id;
    namespc = namespace_;
    radius = radius_;
    max_rv = max_rv_;
    max_tv = max_tv_;
    parent = parent_;
    isChild = true;
    p = p_;
    relativePose = pose_;
}

void Robot::draw() {
  int int_radius = radius * world->inv_res;
  IntPoint p = world->world2grid(poseInWorld().translation);
  cv::circle(world->_display_image, cv::Point(p.y, p.x), int_radius,
             cv::Scalar::all(0), -1);
}

void Robot::timeTick(float dt) {

  Pose motion(tv * dt, 0, rv * dt);
  
  Pose temp = Pose();

  if(this->isChild) {
    temp = this->p->pose_in_parent;
    temp.translation = temp.translation + this->relativePose.translation;
  } else {
    temp = poseInWorld();
  }

  Pose next_pose = temp * motion;
  IntPoint ip = IntPoint();
  
  ip = world->world2grid(next_pose.translation);

  int int_radius = radius * world->inv_res;

  // if(this->id == 2) {
  //   cout << "-------------------------------------------" << endl;
  //   cout << "Pose for ID 2: " << temp << endl;    
  //   cout << "New motion for robot with ID 2: " << motion << endl;
  //   cout << "New ip for robot with ID 2: " << ip << endl;
  //   cout << "-------------------------------------------" << endl;
  // }

  // if(this->id == 0) {
  //   cout << "Pose for ID 0: " << temp << endl;
  //   cout << "New motion for robot with ID 0: " << motion << endl;
  //   cout << "New ip for robot with ID 0: " << ip << endl;
  // }

  if (!world->collides(ip, int_radius)) { // We have not collided, so let's update the position
    if(!isChild) {
      pose_in_parent = next_pose;
    }
  } else { // We have collided. Here we must implement the collision mechanism for stopping both parent/child, but..
    if(isChild) {
      //cout << "Child collided!" << endl;
    } else {
      //cout << "Parent collided!" << endl;
    }
  }
}
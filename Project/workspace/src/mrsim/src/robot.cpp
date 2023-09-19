#include "robot.h"
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using namespace std;

Robot::Robot(float radius_, shared_ptr<World> w_, const Pose& pose_)
    : WorldItem(w_, pose_) {
  radius = radius_;
}

Robot::Robot(float radius_, shared_ptr<WorldItem> p_, const Pose& pose_)
    : WorldItem(p_, pose_) {
  radius = radius_;
}

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, shared_ptr<World> w_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(w_, pose_), nh("~") {
    id = id_;
    type = type_;
    frame_id = frame_id_;
    namespc = namespace_;
    radius = radius_;
    max_rv = max_rv_;
    max_tv = max_tv_;
    parent = parent_;
    isChild = false;
    w = w_;
    relativePose = pose_;
    
    // Initialize the Odometry topic name based on the robot's ID and namespace
    odom_topic = "/" + namespace_ + "/odom";

    // Initialize the Odometry publisher
    robotOdometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
        
}

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, shared_ptr<WorldItem> p_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(p_, pose_), nh("~") {
    id = id_;
    type = type_;
    frame_id = frame_id_;
    namespc = namespace_;
    radius = radius_;
    max_rv = max_rv_;
    max_tv = max_tv_;
    parent = parent_;
    isChild = true;
    p = p_;
    relativePose = pose_;

    // Initialize the Odometry topic name based on the robot's ID and namespace
    odom_topic = "/" + namespace_ + "/odom";

    // Initialize the Odometry publisher
    robotOdometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
}

void Robot::draw() {
  int int_radius = radius * world->inv_res;
  IntPoint p = world->world2grid(poseInWorld().translation);
  cv::circle(world->_display_image, cv::Point(p.y, p.x), int_radius,
             cv::Scalar::all(0), -1);
}

void Robot::timeTick(float dt) {

  // Populate the Odometry message and publish it
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";

  Pose motion(tv * dt, 0, rv * dt);
  
  Pose temp = Pose();

  /**
   * If this robot is a children of another robot, then we get the namespace (Robot_1 or Robot_2 etc.)
   * and we update its pose based on the parent one + its relative pose.
   * 
   * If this robot is not a children, it means is a single robot with no parent,
   * so simply take the poseInWorld() and namespace from constructor.
  */
  if(this->isChild) {
    shared_ptr<Robot> parentRobotPointer = dynamic_pointer_cast<Robot>(this->p);
    
    // Get the namespace (could be for example "robot_0/robot_1"), but get only the
    // last bit in order to update the child_frame_id msg.
    stringstream ss(this->namespc);
    string lastPart;

    while (getline(ss, lastPart, '/')) { // Get last pasrt
    }

    odom.child_frame_id = lastPart;
    temp = this->p->pose_in_parent;
    temp.translation = temp.translation + this->relativePose.translation;
  } else {
    odom.child_frame_id = this->namespc;
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

  // Populate the odometry data here (position and orientation)
  odom.pose.pose.position.x = next_pose.translation.x;
  odom.pose.pose.position.y = next_pose.translation.y;
  odom.pose.pose.orientation.x = next_pose.theta; // Assuming theta represents orientation

  // Publish the Odometry message
  robotOdometryPublisher.publish(odom);

}

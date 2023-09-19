#pragma once
#include "world.h"

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class Robot : public WorldItem {
 public:
  Robot(float radius_, shared_ptr<World> w_, const Pose& pose_ = Pose());

  Robot(float radius_, shared_ptr<WorldItem> p_, const Pose& pose_ = Pose());

  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_, 
        shared_ptr<World> w_, const Pose& pose_ = Pose(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_,
         shared_ptr<WorldItem> p_, const Pose& pose_ = Pose(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  void draw() override;
  void timeTick(float dt) override;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  int id;
  string type;
  string frame_id;
  string namespc;

  float radius;

  int parent;

  Pose relativePose;

  float tv = 0;
  float rv = 0;
  float max_rv;
  float max_tv;

  shared_ptr<World> w = nullptr;
  shared_ptr<WorldItem> p = nullptr;

  bool isChild;

  ros::NodeHandle nh;                      // Handler for Robot
  ros::Publisher robotOdometryPublisher;   // Publisher
  ros::Subscriber cmdVelSubscriber;        // Subscriber for cmd_vel topic
  nav_msgs::Odometry odom;                 // Odometry message
  string odom_topic;                       // Odometry topic
  string cmdVel_topic;                     // cmd_vel topic
};
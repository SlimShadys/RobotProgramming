#pragma once
#include "world.h"

#include <iostream>
#include <string>

#include "types.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h> 

using namespace std;

class Robot : public WorldItem {
 public:
  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_, 
        shared_ptr<World> w_, const Pose& pose_ = Pose::Identity(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_,
         shared_ptr<WorldItem> p_, const Pose& pose_ = Pose::Identity(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  void draw() override;
  void timeTick(float dt) override;

  void transformRobot();
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  int id;
  string type;
  string namespc;
  string parentFrameID;
  string frame_id;

  float radius;

  int parent;

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
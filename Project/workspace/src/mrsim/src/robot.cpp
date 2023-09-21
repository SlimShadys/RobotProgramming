#include "robot.h"
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, shared_ptr<World> w_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(w_, pose_, frame_id_), nh("~") {
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
    parentFrameID = w_->worldFrameID; // "map"
    
    // Initialize the Odometry/cmdVel topic name based on the robot namespace
    odom_topic = "/" + namespace_ + "/odom";
    cmdVel_topic = "/" + namespace_ + "/cmd_vel";

    robotOdometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000); // Initialize the Odometry publisher
    cmdVelSubscriber = nh.subscribe(cmdVel_topic, 10, &Robot::cmdVelCallback, this); // Initialize the cmdVel Subscriber
}

Robot::Robot(int id_, string type_, string frame_id_, string namespace_,
            float radius_, shared_ptr<WorldItem> p_, const Pose &pose_,
            float max_rv_, float max_tv_, int parent_):
        WorldItem(p_, pose_, frame_id_), nh("~"){
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
    parentFrameID = p->itemFrameID;// Anything outside "map"

    // Initialize the Odometry/cmdVel topic name based on the robot namespace
    odom_topic = "/" + namespace_ + "/odom";
    cmdVel_topic = "/" + namespace_ + "/cmd_vel";

    robotOdometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000); // Initialize the Odometry publisher
    cmdVelSubscriber = nh.subscribe(cmdVel_topic, 10, &Robot::cmdVelCallback, this); // Initialize the cmdVel Subscriber
}

void Robot::draw() {
  int int_radius = radius * world->inv_res;
  IntPoint p = world->world2grid(poseInWorld().translation);
  cv::circle(world->_display_image, cv::Point(p.y, p.x), int_radius,
             cv::Scalar::all(0), -1);
}

// Function for clamping velocities based on max_rv and max_tv
void clampVelocity(float& vel, float maxVel, const string& message) {
  if (vel > maxVel) {
    vel = maxVel;
    ROS_WARN_STREAM(message << " Maximum speed reached: " << vel);
  } else if (vel < -maxVel) {
    vel = -maxVel;
    ROS_WARN_STREAM(message << " Maximum speed reached: " << vel);
  }
}

// Callback function for cmd_vel topic
void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Extract linear and angular velocities from the received message
  float linear_vel = msg->linear.x;
  float angular_vel = msg->angular.x;

  clampVelocity(linear_vel, max_tv, "Translation velocity exceeded for robot with ID[" + to_string(id) + "]!");
  clampVelocity(angular_vel, max_rv, "Rotational velocity exceeded for robot with ID[" + to_string(id) + "]!");

  // Update the Robot's velocities
  tv = linear_vel;
  rv = angular_vel;
}

void Robot::timeTick(float dt) {

  // Populate the Odometry message and publish it
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = parentFrameID;
  odom.child_frame_id = frame_id;
  float res = 0.05;

  Pose motion = Pose();

  if(isChild) {
    shared_ptr<Robot> robotPointer = std::dynamic_pointer_cast<Robot>(p); // Get the parent

    motion.translation.x = robotPointer->tv * dt;
    motion.translation.y = 0;
    motion.theta = robotPointer->rv * dt;
  } else {
    motion.translation.x = tv * dt;
    motion.translation.y = 0;
    motion.theta = rv * dt;    
  }
  
  Pose next_pose = pose_in_parent * motion;
  Pose temp = Pose();
  IntPoint ip = IntPoint();

  // if (isChild) {
  //   shared_ptr<Robot> robotPointer = std::dynamic_pointer_cast<Robot>(p); // Get the parent
  //   temp = robotPointer->pose_in_parent;
  //   temp.translation.x = temp.translation.x + motion.translation.x;
  //   temp.translation.y = temp.translation.y + motion.translation.y;
  //   temp.theta = temp.theta + motion.theta;
  //   ip = world->world2grid(next_pose.translation + temp.translation);
  // } else {
  //   ip = world->world2grid(next_pose.translation);
  // }
    
  ip = world->world2grid(next_pose.translation);  
  int int_radius = radius * world->inv_res;

  if (!world->collides(ip, int_radius)) { // We have not collided, so let's update the position
    pose_in_parent = next_pose;      
  } else { // We have collided. Here we must implement the collision mechanism for stopping both parent/child, but..
    if(isChild) {
      //cout << "Child collided!" << endl;
    } else {
      //cout << "Parent collided!" << endl;
    }
  }
  
  geometry_msgs::Quaternion geometry_quaternion;

  if(isChild) {

    shared_ptr<Robot> robotPointer = std::dynamic_pointer_cast<Robot>(p); // Get the parent

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, pose_in_parent.theta);
    tf_quaternion.normalize();

    // Set the components of the geometry_msgs::Quaternion
    geometry_quaternion.x = tf_quaternion.x();
    geometry_quaternion.y = tf_quaternion.y();
    geometry_quaternion.z = tf_quaternion.z();
    geometry_quaternion.w = tf_quaternion.w();

    // Populate the odometry data here (position and orientation)
    odom.pose.pose.position.x = (pose_in_parent.translation.x);
    odom.pose.pose.position.y = (pose_in_parent.translation.y);
  } else {
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, pose_in_parent.theta);
    tf_quaternion.normalize();

    // Set the components of the geometry_msgs::Quaternion
    geometry_quaternion.x = tf_quaternion.x();
    geometry_quaternion.y = tf_quaternion.y();
    geometry_quaternion.z = tf_quaternion.z();
    geometry_quaternion.w = tf_quaternion.w();

    // Populate the odometry data here (position and orientation)
    odom.pose.pose.position.x = pose_in_parent.translation.x;
    odom.pose.pose.position.y = pose_in_parent.translation.y;    
  }
  
  odom.pose.pose.orientation = geometry_quaternion; // Assuming theta represents orientation

  // Publish the Odometry message
  robotOdometryPublisher.publish(odom);
  transformRobot();

}

void Robot::transformRobot() {
  // Get the 2D pose of the robot in world coordinates
  Pose transformation = Pose();
  float res = 0.05;

  transformation.translation.x = pose_in_parent.translation.x;
  transformation.translation.y = pose_in_parent.translation.y;
  transformation.theta = pose_in_parent.theta;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = parentFrameID;
  transform_stamped.child_frame_id = frame_id;

  /**
   * We want to create a quaternion that represents the rotation around the z-axis.
   * We use atan2, which is the correct function to use when we have only one argument
   * (in this case, transformation.theta).
   * It calculates the arctangent of the ratio of its two arguments and correctly handles
   * the case where the denominator is zero. 
   * 
   * This is the right choice for representing 2D rotations.
   * 
   * Quaternion: x, y, z, w
  */
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, transformation.theta);
  rotation.normalize();

  // Let's create a translation vector which will be composed by (x, y, z).
  // Since we are in 2D, z is equal to 0.
  tf2::Vector3 translation(transformation.translation.x, transformation.translation.y, 0.0);

  // Now we create the Transform object, by applying the rotation and translation created before.
  tf2::Transform tf_transform(rotation, translation);

  // Translation part. We get the translation values and put them in the transform_stamped.
  transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
  transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
  transform_stamped.transform.translation.z = tf_transform.getOrigin().z();

  // Rotation part. We get the rotation values and put them in the transform_stamped.
  transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
  transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
  transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
  transform_stamped.transform.rotation.w = tf_transform.getRotation().w();

  // Create a TransformBroadcaster which will send the transform_stamped we created
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_stamped);
}

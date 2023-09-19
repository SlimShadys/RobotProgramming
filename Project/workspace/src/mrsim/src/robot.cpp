#include "robot.h"
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

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
    
    // Initialize the Odometry/cmdVel topic name based on the robot namespace
    odom_topic = "/" + namespace_ + "/odom";
    cmdVel_topic = "/" + namespace_ + "/cmd_vel";

    robotOdometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000); // Initialize the Odometry publisher
    cmdVelSubscriber = nh.subscribe(cmdVel_topic, 10, &Robot::cmdVelCallback, this); // Initialize the cmdVel Subscriber
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
      // Populate the odometry data here (position and orientation)
      odom.pose.pose.position.x = pose_in_parent.translation.x * w->res;
      odom.pose.pose.position.y = pose_in_parent.translation.y * w->res;
      odom.pose.pose.orientation.x = pose_in_parent.theta; // Assuming theta represents orientation
    } else { // We have a children, so let's take p-> | THIS CRASHES IF WE HAVE 3 ROBOTS ON TOP OF EACH OTHER
      // Populate the odometry data here (position and orientation)
      odom.pose.pose.position.x = next_pose.translation.x * p->world->res;
      odom.pose.pose.position.y = next_pose.translation.y * p->world->res;
      odom.pose.pose.orientation.x = next_pose.theta; // Assuming theta represents orientation   
    }
  } else { // We have collided. Here we must implement the collision mechanism for stopping both parent/child, but..
    if(isChild) {
      //cout << "Child collided!" << endl;
    } else {
      //cout << "Parent collided!" << endl;
    }
  }

  // Populate the odometry data here (position and orientation)
  //ROS_INFO_STREAM("ID[" << id << "]: odom.pose.pose.position.x -> " << odom.pose.pose.position.x << endl);
  //ROS_INFO_STREAM("ID[" << id << "]: odom.pose.pose.position.y -> " << odom.pose.pose.position.y << endl);
  //ROS_INFO_STREAM("ID[" << id << "]: odom.pose.pose.orientation.x -> " << odom.pose.pose.orientation.x << endl);

  // Publish the Odometry message
  robotOdometryPublisher.publish(odom);

}

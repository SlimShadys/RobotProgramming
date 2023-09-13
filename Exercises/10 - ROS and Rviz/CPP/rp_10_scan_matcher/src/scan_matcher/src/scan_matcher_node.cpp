#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include "icp/eigen_icp_2d.h"
#include "ros_bridge.h"

// Define node I/O parameters
const std::string TOPIC_SCAN = "scan";
const std::string TOPIC_ODOM = "odom";
const std::string FRAME_WORLD = "map";

ros::Publisher odom_pub; // We define a publisher odom_pub.
std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Vector2f>> scan_fixed; // Fixed scan definition
Eigen::Isometry2f laser_in_world = Eigen::Isometry2f::Identity(); // Laser in world

/**
 *  ---- SCAN CALLBACK ----
*/
void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);        // Scan callback definition (Header)
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg_) {  // Scan callback implementation
  /**
   * @brief Task of this callback is to solve a frame-to-frame lidar odometry
   * problem frame-to-frame means that we are estimating the relative ego-motion
   * of the lidar using two frames with sufficient overlap. For simplicity, we
   * are using two sequential scans which provide a lot of overlap.
   */
  // Convert sensor_msgs::LaserScan to std::vector<Eigen::Vector2f>
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> scan;
  scan2eigen(msg_, scan);

  // Fix the first scan and stop the callback
  if (scan_fixed.size() == 0) {
    scan_fixed = scan;
    ROS_INFO("Storing scan as fixed.");
    return;
  }

  // Apply ICP to estimate new pose
  ICP solver(scan_fixed, scan, 4);
  solver.run(100);

  // Update current laser_in_world estimate
  laser_in_world = laser_in_world * solver.X();

  // Prepare tf and odom messages
  geometry_msgs::TransformStamped tf_msg;
  nav_msgs::Odometry odom_msg;

  isometry2transformStamped(laser_in_world, tf_msg, FRAME_WORLD,
                            msg_->header.frame_id, msg_->header.stamp);
  transformStamped2odometry(tf_msg, odom_msg);

  // Send messages
  odom_pub.publish(odom_msg);
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(tf_msg);
  ROS_INFO("Scan [points:%ld, chi:%f, pos=(%f, %f)]", scan.size(), solver.chi(),
           laser_in_world.translation().x(), laser_in_world.translation().y());

  // Update the fixed scan with the current one
  scan_fixed = scan;
}

/**
 * Main function
*/
int main(int argc, char** argv) {

  // Initialize ROS system. Our node is called "scan_matcher_node"
  ros::init(argc, argv, "scan_matcher_node");

  // Create a NodeHandle to manage the node.
  // The "~" is used to configure the NodeHandle namespace equal to
  // the node's name.
  ros::NodeHandle nh("~");

  // Subscribe to the topic "~/scan" (globally resolved as "scan_matcher_node/scan").
  //
  // The topic should contain sensor_msgs::LaserScan messages.
  // We configure the scan_callback function to handle the received messages.
  ros::Subscriber scan_subscriber = nh.subscribe(TOPIC_SCAN, 10, scan_callback);

  // Publish on topic "~/odom" (globally resolved as "scan_matcher/odom").
  // The topic will contain nav_msgs::Odometry messages
  odom_pub = nh.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 10);

  ROS_INFO("Node started. Waiting for scans.");
  ros::spin();

  return 0;
}

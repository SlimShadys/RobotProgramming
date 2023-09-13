#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Dense>
#include <vector>

void scan2eigen(const sensor_msgs::LaserScan::ConstPtr& msg_,
                std::vector<Eigen::Vector2f,
                            Eigen::aligned_allocator<Eigen::Vector2f>>& dest_);

void isometry2transformStamped(const Eigen::Isometry2f& pose_,
                               geometry_msgs::TransformStamped& msg_,
                               const std::string& frame_id_,
                               const std::string& child_frame_id_,
                               const ros::Time& stamp_);

void transformStamped2odometry(const geometry_msgs::TransformStamped& msg_,
                               nav_msgs::Odometry& odom_);
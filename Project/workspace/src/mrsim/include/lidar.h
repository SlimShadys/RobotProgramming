#pragma once
#include "world.h"

#include <iostream>
#include <string>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

using namespace std;

class Lidar: public WorldItem {
public:
  Lidar(float fov_, float max_range_, int num_beams_, shared_ptr<World> w, const Pose& pose_=Pose());

  Lidar(float fov_, float max_range_, int num_beams_, shared_ptr<WorldItem> p_, const Pose& pose_=Pose());

  Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<World> w, 
        const Pose& pose_=Pose(), float max_range_ = 10.0, int num_beams_ = 360, int parent_ = -1);

  Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<WorldItem> p_, 
        const Pose& pose_=Pose(), float max_range_ = 10.0, int num_beams_ = 360, int parent_ = -1);

  ~Lidar();

  void timeTick(float dt);

  void draw();

  void publishLidarScan(); // Method to publish LaserScan data
  void transformLidar();

  int id;
  string type;
  string frame_id;
  string namespc;
  string parentFrameID;

  int parent;

  float fov;
  float max_range;
  int num_beams;
  float *ranges;

  bool isChild;

  ros::NodeHandle nh;                      // Handler for Lidar
  ros::Publisher lidarScanPublisher;       // Publisher
  sensor_msgs::LaserScan lscan;            // LaserScan message
  string lscan_topic;                      // LaserScan topic
};

#include "lidar.h"

#include <iostream>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/LaserScan.h>

using namespace std;

Lidar::Lidar(float fov_, float max_range_, int num_beams_, shared_ptr<World> w, const Pose& pose_):
  WorldItem(w,pose_),
  fov(fov_),
  max_range(max_range_),
  num_beams(num_beams_){
  ranges = new float[num_beams];
  }

Lidar::Lidar(float fov_, float max_range_, int num_beams_, shared_ptr<WorldItem> p_, const Pose& pose_):
  WorldItem(p_,pose_),
  fov(fov_),
  max_range(max_range_),
  num_beams(num_beams_){
  ranges = new float[num_beams];
  }

  Lidar::Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<World> w, 
  const Pose &pose_, float max_range_, int num_beams_, int parent_):
  WorldItem(w, pose_), nh("~")
  {
    id = id_;
    type = type_;
    frame_id = frame_id_;
    namespc = namespace_;
    fov = fov_;
    max_range = max_range_;
    num_beams = num_beams_;
    parent = parent_;
    ranges = new float[num_beams];

    // Initialize the LidarScan topic name based on the Lidar namespace
    lscan_topic = "/" + namespace_ + "/base_scan";

    // Initialize the LidarScan publisher
    lidarScanPublisher = nh.advertise<sensor_msgs::LaserScan>(lscan_topic, 1000);

  }


  Lidar::Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<WorldItem> p_, 
  const Pose &pose_, float max_range_, int num_beams_, int parent_):
  WorldItem(p_, pose_), nh("~")
  {
    id = id_;
    type = type_;
    frame_id = frame_id_;
    namespc = namespace_;
    fov = fov_;
    max_range = max_range_;
    num_beams = num_beams_;
    parent = parent_;
    ranges = new float[num_beams];

    // Initialize the LidarScan topic name based on the Lidar namespace
    lscan_topic = "/" + namespace_ + "/base_scan";

    // Initialize the LidarScan publisher
    lidarScanPublisher = nh.advertise<sensor_msgs::LaserScan>(lscan_topic, 1000);
  }

Lidar::~Lidar() {
  if (ranges)
    delete [] ranges;
}


void Lidar::timeTick(float dt) {
  Pose piw=poseInWorld();
  IntPoint origin=world->world2grid(piw.translation);
  if (! world->inside(origin))
    return;

  float d_alpha=fov/num_beams;
  float alpha=piw.theta-fov/2;
  float int_range=max_range*world->inv_res;
    
  for (int i=0; i<num_beams; ++i) {
    IntPoint endpoint;
    ranges[i]=max_range;
    bool result=world->traverseBeam(endpoint, origin, alpha, int_range);
    if (result) {
      IntPoint delta=endpoint-origin;
      ranges[i]=sqrt(delta*delta)*world->res;
    }
    alpha += d_alpha;
  }
  publishLidarScan();
}

void Lidar::draw() {
  Pose piw=poseInWorld();
  IntPoint origin=world->world2grid(piw.translation);
  
  if (! world->inside(origin))
    return;

  float d_alpha=fov/num_beams;
  float alpha=-fov/2;
  for (int i=0; i<num_beams; ++i) {
    float r = ranges[i];
    Point p_lidar(r*cos(alpha), r*sin(alpha));
    Point p_world = piw*p_lidar;
    IntPoint epi=world->world2grid(p_world);
    cv::line(world->_display_image, cv::Point(origin.y, origin.x), cv::Point(epi.y, epi.x), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }   
}

void Lidar::publishLidarScan() {
  // Fill in the LaserScan message with Lidar data
  lscan.header.stamp = ros::Time::now();
  lscan.header.frame_id = "map";
  lscan.angle_min = -(this->fov) / 2;                   // Start angle of the scan
  lscan.angle_max = this->fov / 2;                      // End angle of the scan
  lscan.angle_increment = this->fov / this->num_beams;  // Angle increment between each measurement
  lscan.time_increment = 0.0;                           // Time between each scan point (not used)
  lscan.scan_time = 0.0;                                // Time it took to complete one scan (not used)
  lscan.range_min = 0.0;                                // Minimum range value
  lscan.range_max = this->max_range;                    // Maximum range value
  lscan.ranges.clear();

  // Fill in the range values (assuming 'ranges' is already populated)
  for (int i = 0; i < this->num_beams; ++i) {
    lscan.ranges.push_back(this->ranges[i]);
  }

  // Publish the LaserScan message
  lidarScanPublisher.publish(lscan);
}

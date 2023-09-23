#include "lidar.h"
#include "types.h"

#include <iostream>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/LaserScan.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

Lidar::Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<World> w, 
const Pose &pose_, float max_range_, int num_beams_, int parent_):
WorldItem(w, pose_, frame_id_), nh("~")
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
  parentFrameID = w->worldFrameID; // "map"

  // Initialize the LidarScan topic name based on the Lidar namespace
  lscan_topic = "/" + namespace_ + "/base_scan";

  // Initialize the LidarScan publisher
  lidarScanPublisher = nh.advertise<sensor_msgs::LaserScan>(lscan_topic, 1000);

}

Lidar::Lidar(int id_, string type_, string frame_id_, string namespace_, float fov_, shared_ptr<WorldItem> p_, 
const Pose &pose_, float max_range_, int num_beams_, int parent_):
WorldItem(p_, pose_, frame_id_), nh("~")
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
  parentFrameID = p_->itemFrameID; // Anything outside "map"

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
  Pose piw = poseInWorld();
  
  IntPoint origin=world->world2grid(piw.translation());

  if (! world->inside(origin))
    return;

  float d_alpha = fov / num_beams;
  float alpha = (Rotation(piw.linear()).angle()) - fov / 2;
  float int_range = max_range * world->inv_res;
    
  for (int i = 0; i < num_beams; ++i) {
    IntPoint endpoint;
    ranges[i] = max_range;
    bool result = world->traverseBeam(endpoint, origin, alpha, int_range);
    if (result) {
      IntPoint delta = endpoint-origin;
      ranges[i] = delta.norm() * world->res;
    }
    alpha += d_alpha;
  }
  publishLidarScan();
  transformLidar();
}

void Lidar::draw() {
  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation());
  
  if (!world->inside(origin))
    return;

  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i) {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar;
    IntPoint epi = world->world2grid(p_world);
    cv::line(world->_display_image, cv::Point(origin.y(), origin.x()), cv::Point(epi.y(), epi.x()), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }   
}

void Lidar::publishLidarScan() {
  // Fill in the LaserScan message with Lidar data
  lscan.header.stamp = ros::Time::now();
  lscan.header.frame_id = this->itemFrameID;
  lscan.angle_min = -fov / 2;                   // Start angle of the scan
  lscan.angle_max = fov / 2;                      // End angle of the scan
  lscan.angle_increment = fov / num_beams;        // Angle increment between each measurement
  lscan.time_increment = 0.0;                     // Time between each scan point (not used)
  lscan.scan_time = 0.0;                          // Time it took to complete one scan (not used)
  lscan.range_min = 0.0;                          // Minimum range value
  lscan.range_max = max_range;                    // Maximum range value
  lscan.ranges.clear();

  // Fill in the range values (assuming 'ranges' is already populated)
  for (int i = 0; i < num_beams; ++i) {
    lscan.ranges.push_back(ranges[i]);
  }

  // Publish the LaserScan message
  lidarScanPublisher.publish(lscan);
}

void Lidar::transformLidar() {
  // We transform from lidar to robot
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = parentFrameID;
  transform_stamped.child_frame_id = frame_id;
  transform_stamped.header.stamp = ros::Time::now();

  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  q.normalize();

  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  br.sendTransform(transform_stamped);
}
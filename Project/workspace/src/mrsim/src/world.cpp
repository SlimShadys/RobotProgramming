#include "world.h"
#include "robot.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

using namespace std;

World::World() {}

WorldItem::WorldItem(shared_ptr<World> w_, const Pose& p_, string frame_id_)
    : world(w_), parent(nullptr), pose_in_parent(p_), itemFrameID(frame_id_) {
  if (world) world->add(this);
}

WorldItem::WorldItem(shared_ptr<WorldItem> parent_, const Pose& p, string frame_id_)
    : world(parent_->world), parent(parent_), pose_in_parent(p), itemFrameID(frame_id_) {
  if (world) world->add(this);
}

WorldItem::~WorldItem() {}

Pose WorldItem::poseInWorld() {
  if (!parent) return pose_in_parent;
  return parent->poseInWorld() * pose_in_parent;
}

void World::loadFromImage(const string filename)
{
  cerr << "Loading [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0)
  {
    throw runtime_error("Unable to load image");
  }
  cv::cvtColor(m, _display_image, cv::COLOR_BGR2GRAY);
  size = _display_image.rows * _display_image.cols;
  grid = vector<uint8_t>(size, 0x00);
  rows = _display_image.rows;
  cols = _display_image.cols;
  memcpy(grid.data(), _display_image.data, size);
}

bool World::collides(const IntPoint &p, const int &radius) const
{
  if (!inside(p)) return true;
  int r2 = radius * radius;
  for (int r = -radius; r <= radius; ++r)
  {
    for (int c = -radius; c <= radius; ++c)
    {
      IntPoint off(r, c);
      if (off.squaredNorm() > r2) continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (!inside(p_test)) return true;
      if (at(p_test) < 127) return true; // Pixels darker than 127 (0 black | 225 white) are marked as obstacles
    }
  }
  return false;
}

void World::draw() {
  for (const auto item : _items) item->draw();

  cv::imshow("Map", _display_image);
  memcpy(_display_image.data, grid.data(), size); // Clean the display image
}

void World::timeTick(float dt) {
  for (const auto item : _items) item->timeTick(dt);
}

void World::add(WorldItem* item) { _items.push_back(item); }

bool World::traverseBeam(IntPoint &endpoint, const IntPoint &origin,
                         const float angle, const int max_range) {
  Point p0 = origin.cast<float>();
  const Point dp(cos(angle), sin(angle));
  int range_to_go = max_range;

  while (range_to_go > 0) {
    endpoint = IntPoint(p0.x(), p0.y());
    if (!inside(endpoint)) return false;
    if (at(endpoint) < 127) return true;

    // // For all items in the world
    // for (auto item: this->_items) {
    //   if (item->itemFrameID.find("frame_lidar") == 0) continue; // We don't want to check if a Lidar is hitting a Lidar

    //   Robot* robotPointer = dynamic_cast<Robot*>(item);

    //   IntPoint piw_item = this->world2grid(robotPointer->poseInWorld().translation()); // Get the pose of the robot.
      
    //   // Assuming the robot's position is stored in piw_item, and it has a radius 'robotRadius'.
    //   // Calculate the distance from the lidar endpoint to the robot's center.
    //   float distance_to_robot = (piw_item.cast<float>() - p0).norm();

    //   if (distance_to_robot < robotPointer->radius * inv_res) { // Lidar beam intersects with the robot.
    //     //cout << "Laser is hitting the robot n." << robotPointer->id << endl;
    //     return false;
    //   }
    // }

    p0 = p0 + dp;
    --range_to_go;
  }

  return true;
}

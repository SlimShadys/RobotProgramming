#include "world.h"

#include <stdexcept>

using namespace std;

World::World() { memset(items, 0, sizeof(WorldItem*) * MAX_ITEMS); }

bool World::collides(const IntPoint& p, const int& radius) const {
  if (!inside(p)) return true;
  int r2 = radius * radius;
  for (int r = -radius; r <= radius; ++r) {
    for (int c = -radius; c <= radius; ++c) {
      IntPoint off(r, c);
      if (off * off > r2) continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (!inside(p_test)) return true;

      // cerr << "r: " << r << " c: " << c << " val: " << (int) at(p_test) <<
      // endl;
      if (at(p_test) < 127) return true;
    }
  }
  return false;
}

void World::loadFromImage(const char* filename) {
  cerr << "loading [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0) {
    throw std::runtime_error("unable to load image");
  }
  cv::cvtColor(m, _display_image, cv::COLOR_BGR2GRAY);
  size = _display_image.rows * _display_image.cols;
  grid = new uint8_t[_display_image.rows * _display_image.cols];
  rows = _display_image.rows;
  cols = _display_image.cols;
  memcpy(grid, _display_image.data, size);
}

float angle = 0;
void World::draw() {
  memcpy(_display_image.data, grid, size);
  for (int i = 0; i < num_items; ++i) {
    if (items[i]) items[i]->draw();
  }

  // IntPoint origin(rows/2, cols/2);
  // IntPoint epi;
  // bool result=traverseBeam(epi, origin, angle, 400);
  // angle += M_PI/180;
  // cv::line(_display_image, cv::Point(origin.y, origin.x), cv::Point(epi.y,
  // epi.x), cv::Scalar(127,127,127), 1);
  cv::imshow("map", _display_image);
}

void World::timeTick(float dt) {
  for (int i = 0; i < num_items; ++i)
    if (items[i]) items[i]->timeTick(dt);
}

bool World::add(WorldItem& item) {
  if (num_items < MAX_ITEMS) {
    items[num_items] = &item;
    ++num_items;
    return true;
  }
  return false;
}

bool World::traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                         const float angle, const int max_range) {
  Point p0(origin.x, origin.y);
  const Point dp(cos(angle), sin(angle));
  int range_to_go = max_range;
  while (range_to_go > 0) {
    endpoint = IntPoint(p0.x, p0.y);
    if (!inside(endpoint)) return false;
    if (at(endpoint) < 127) return true;
    p0 = p0 + dp;
    --range_to_go;
  }
}

WorldItem::WorldItem(World* w_, const Pose& p) {
  parent = 0;
  world = w_;
  pose = p;
  if (world) {
    bool result = world->add(*this);
    if (!result) throw std::runtime_error("world full");
  }
}

WorldItem::WorldItem(WorldItem* p_, const Pose& p) {
  parent = p_;
  world = parent->world;
  pose = p;
  if (world) {
    bool result = world->add(*this);
    if (!result) throw std::runtime_error("world full");
  }
}

// TODO
// Clear the world and any items linked to this instance
//  to avoid segmentation faults
WorldItem::~WorldItem() {
  // If item is not linked to any world, then nothing to do
  // TODO

  // Remove this from world->items
  // TODO

  // If a world item is linked to 'this',  then
  //  replace 'this' with 'this->parent' to
  //  remove 'this' from the items chain.
  // TODO
}

Pose WorldItem::poseInWorld() {
  if (!parent) return pose;
  return parent->poseInWorld() * pose;
}

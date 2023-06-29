#include "world.h"

#include <stdexcept>

using namespace std;

World::World() { memset(items, 0, sizeof(WorldItem *) * MAX_ITEMS); }

bool World::collides(const IntPoint &p, const int &radius) const
{
  if (!inside(p))
    return true;
  int r2 = radius * radius;
  for (int r = -radius; r <= radius; ++r)
  {
    for (int c = -radius; c <= radius; ++c)
    {
      IntPoint off(r, c);
      if (off * off > r2)
        continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (!inside(p_test))
        return true;
      if (at(p_test) < 127)
        return true;
    }
  }
  return false;
}

void World::loadFromImage(const char *filename)
{
  cerr << "loading [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0)
  {
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
void World::draw(float rotationV, float translationV)
{
  memcpy(_display_image.data, grid, size);
  for (int i = 0; i < num_items; ++i)
  {
    if (items[i])
      items[i]->draw();
  }

  cv::Point text_position(0, 30);//Declaring the text position//
  cv::Point text_position2(0, 60);//Declaring the text position//
  cv::Scalar font_Color(0, 0, 0);//Declaring the color of the font//

  int font_size = 1;
  int font_weight = 1;

  cv::putText(_display_image, "Rotation velocity   : " + to_string(rotationV), text_position, 0, font_size, font_Color, font_weight);
  cv::putText(_display_image, "Translation velocity : " + to_string(translationV), text_position2, 0, font_size, font_Color, font_weight);

  cv::imshow("map", _display_image);
}

void World::timeTick(float dt)
{
  for (int i = 0; i < num_items; ++i)
    if (items[i])
      items[i]->timeTick(dt);
}

bool World::add(WorldItem &item)
{
  if (num_items < MAX_ITEMS)
  {
    items[num_items] = &item;
    ++num_items;
    return true;
  }
  return false;
}

bool World::traverseBeam(IntPoint &endpoint, const IntPoint &origin,
                         const float angle, const int max_range) {
  Point p0(origin.x, origin.y);
  const Point dp(cos(angle), sin(angle));
  int range_to_go = max_range;
  while (range_to_go > 0) {
    endpoint = IntPoint(p0.x, p0.y);
    if (!inside(endpoint))
      return false;
    if (at(endpoint) < 127)
      return true;
    p0 = p0 + dp;
    --range_to_go;
  }
  return false;
}

WorldItem::WorldItem(World *w_, const Pose &p)
{
  parent = 0;
  world = w_;
  pose = p;
  if (world)
  {
    bool result = world->add(*this);
    if (!result)
      throw std::runtime_error("world full");
  }
}

WorldItem::WorldItem(WorldItem *p_, const Pose &p)
{
  parent = p_;
  world = parent->world;
  pose = p;
  if (world)
  {
    bool result = world->add(*this);
    if (!result)
      throw std::runtime_error("world full");
  }
}

WorldItem::~WorldItem()
{
  // Let's first check if the world is not null
  if(world) {
    // Cycle throughout the whole items of the world
    for (int i = 0; i < world->num_items; ++i) {
      if(!world->items[i]) {
        // Still no link. Let's move to the next one
        continue;
      } else if (world->items[i] == this) { // I found a proper item. Let's check if it's linked to this instance
        world->items[i] = 0; // Remove this from world->items
      } else {
        // This item is not linked to this instance, but it's still part of the world
      }

      // If a world item is linked to 'this',  then
      //  replace 'this' with 'this->parent' to
      //  remove 'this' from the items chain.
      if (world->items[i]->parent == this) {
        world->items[i]->parent = parent;
      }
    }
  } else {
    // Nothing to do here since the item is not linked to any world.
  }
}

Pose WorldItem::poseInWorld()
{
  if (!parent)
    return pose;
  return parent->poseInWorld() * pose;
}

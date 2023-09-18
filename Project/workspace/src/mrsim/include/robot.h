#pragma once
#include "world.h"
#include <iostream>
#include <string>

using namespace std;

class Robot : public WorldItem {
 public:
  Robot(float radius_, std::shared_ptr<World> w_, const Pose& pose_ = Pose());

  Robot(float radius_, std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose());

  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_, 
        std::shared_ptr<World> w_, const Pose& pose_ = Pose(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  Robot(int id_, string type_, string frame_id_, string namespace_, float radius_,
         std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose(), float max_rv_ = 100.0, float max_tv_ = 100.0, int parent_ = -1);

  void draw() override;
  void timeTick(float dt) override;

  int id;
  string type;
  string frame_id;
  string namespc;

  float radius;

  int parent;

  Pose relativePose;

  float tv = 0;
  float rv = 0;
  float max_rv;
  float max_tv;

  std::shared_ptr<World> w = nullptr;
  std::shared_ptr<WorldItem> p = nullptr;

  bool isChild;
};
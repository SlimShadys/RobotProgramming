#pragma once
#include "world.h"

class Robot : public WorldItem {
 public:
  Robot(float radius_, std::shared_ptr<World> w, const Pose& pose_ = Pose());

  Robot(float radius_, std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose());

  void draw() override;
  void timeTick(float dt) override;
  float radius;
  float tv = 0, rv = 0;
};

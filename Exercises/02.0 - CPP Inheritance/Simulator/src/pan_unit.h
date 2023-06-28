#include "world.h"

class PanUnit : public WorldItem {
 public:
  PanUnit(WorldItem* parent_, float max_rv_, const Pose& pose_ = Pose());

  void draw() override;
  void timeTick(float dt) override;

  void setAngle(float ang_);

  Pose initial_pose;

  float current_angle;
  float desired_angle;
  float max_rv;
};
#include "pan_unit.h"

// Initialize the class.
// Remember that the pan unit is controlled in relative angle
// Store the initial (parent_ -> object) as base pose
PanUnit::PanUnit(WorldItem* parent_, float max_rv_, const Pose& pose_)
    : WorldItem(parent_, pose_) /*TODO*/ {}

void PanUnit::setAngle(float ang_) {
  // Update the desired angle to be reached
}

void PanUnit::timeTick(float dt) {
  // Compute error as desired_angle - current_angle
  // TODO

  // Apply bound on error to achiecve slower motion
  // TODO

  // Bound depends by the maximum rotational velocity of the unit
  //  and the simulator time increment dt
  // TODO

  // Add the error to the current unit's angle and update the pose
  // TODO
}

void PanUnit::draw() {
  // Notin to see here
}
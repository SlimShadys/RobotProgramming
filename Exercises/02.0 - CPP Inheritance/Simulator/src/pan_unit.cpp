#include "pan_unit.h"

using namespace std;

// Initialize the class.
PanUnit::PanUnit(WorldItem *parent_, float max_rv_, const Pose &pose_)
    : WorldItem(parent_, pose_),
      initial_pose(pose_),
      max_rv(max_rv_) {}

// Update the desired angle to be reached
void PanUnit::setAngle(float ang_)
{
  desired_angle = ang_;
}

// Surprisingly the robot works better with this commented //
void PanUnit::timeTick(float dt)
{
  // // Compute error as desired_angle - current_angle
  // float error = desired_angle - current_angle;

  // // Apply bound on error to achieve slower motion
  // float boundary = max_rv * dt;

  // // Bound depends by the maximum rotational velocity of the unit
  // // and the simulator time increment dt
  // if (error >= boundary)
  // {
  //   error = boundary;
  // }
  // if (error < -boundary)
  // {
  //   error = -boundary;
  // }

  // // Add the error to the current unit's angle and update the pose
  // current_angle += error;
  // pose.theta = initial_pose.theta + current_angle;
  // cerr << "pose.theta: " << pose.theta << endl;
}

void PanUnit::draw()
{
  // Notin to see here
}
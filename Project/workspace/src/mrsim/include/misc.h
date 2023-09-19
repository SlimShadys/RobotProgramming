#pragma once

#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <iostream>
#include <jsoncpp/json/json.h> 

using namespace std;

// Todo - We are passing a simple Robot for testing
void getRobotsAndLidars(shared_ptr<World> worldSharedPointer, Json::Value root);

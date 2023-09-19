#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "misc.h"

#include <opencv2/highgui.hpp>
#include <jsoncpp/json/json.h>

using namespace std;

int main(int argc, char** argv) {

  // We might want a function that returns both maps for Lidars/Robots
  //map<int, shared_ptr<Robot>> robotsPointersMap; 
  //map<int, shared_ptr<Lidar>> lidarsPointersMap;

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  /*
  * Check if a JSON file path is provided as a command-line argument
  * If not, then we simply terminate our node.
  */
  if (argc != 2) {
      ROS_ERROR("Usage: rosrun mrsim mrsim_node <config_file.json>");
      return 1;
  }

  // We get the path of the JSON file from the arguments
  const string jsonFilePath = argv[1];

  // Read and parse the JSON file
  Json::Value root;
  Json::Reader reader;
  ifstream jsonFile(("/home/lattinone/RobotProgramming/Project/workspace/src/mrsim/test_data/" + jsonFilePath).c_str(), ifstream::binary); // We need a way to remove absolute path
  
  // Let's check if the JSON is properly written and decoded.
  if (!jsonFile.good()) {
      ROS_ERROR_STREAM("Failed to open JSON file: " << jsonFilePath << "\nPlease check your JSON config.");
      return 1;
  }

  if (!reader.parse(jsonFile, root)) {
      ROS_ERROR_STREAM("Failed to parse JSON file: " << reader.getFormattedErrorMessages() << "\nPlease check your JSON config.");
      jsonFile.close();
      return 1;
  }

  // We access the map file
  string mapFileName = root["map"].asString();

  // Create a World object
  World w = World();
  w.loadFromImage(("/home/lattinone/RobotProgramming/Project/workspace/src/mrsim/test_data/" + mapFileName).c_str()); // We need a way to remove absolute path

  /**
   * Create a shared pointer to the World.
   * We instantiate the World in the same way we instantiate Robots/Lidars in getRobotsAndLidars().
   * Check comments there.
   * */
  shared_ptr<World> worldSharedPtr(&w,
                                        [](World* w) {
                                            // Custom cleanup actions here, if needed
                                            //delete w; // Clean up the Robot object
                                        });

  // Get Robots and Lidars
  shared_ptr<Robot> robotP = getRobotsAndLidars(worldSharedPtr, root);
  
  float delay = 0.07;
  int k;

  while (ros::ok()) {

    ros::spinOnce();

    // run a simulation iteration
    w.timeTick(delay);
    w.draw();

    k = cv::waitKeyEx(delay*1000)&255;
    switch (k) {
      case 81:  // Left arrow
        robotP->rv += 0.10;
        break;  
      case 82:  // Up arrow
        robotP->tv += 0.2;
        break;  
      case 83:  // Right arrow
        robotP->rv -= 0.10;
        break;  
      case 84:  // Down arrow
        robotP->tv -= 0.2;
        break;  
      case ' ':  // Spacebar
        robotP->tv = 0;
        robotP->rv = 0;
        robotP->pose_in_parent.theta = 0;
        break;
      case 27:  // Esc
        return 0;
      default:
        //robotP->rv = 0;
        continue;
    }

    // visualize the simulation
    cv::waitKey(100);
  }

  return 0;
}
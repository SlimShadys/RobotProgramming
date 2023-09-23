#include <ros/ros.h>
#include <iostream>
#include <fstream>

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
  getRobotsAndLidars(worldSharedPtr, root);
  
  float delay = 0.07;
  int k;

  while (ros::ok()) {

    ros::spinOnce();

    // run a simulation iteration
    w.timeTick(delay);
    w.draw();

    k = cv::waitKeyEx(1)&255;
    if (k == 27) {     // Esc
      ros::shutdown(); // Terminate the ROS node
      break; 
    } 
  }

  cv::destroyAllWindows(); // Destroy the CV2 window used for drawing
  return 0;
}
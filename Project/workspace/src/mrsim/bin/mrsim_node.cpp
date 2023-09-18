#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include "world.h"
#include "robot.h"
#include "lidar.h"

#include <opencv2/highgui.hpp>
#include <jsoncpp/json/json.h>

using namespace std;

int main(int argc, char** argv) {

  std::map<int, std::shared_ptr<Robot>> robotsPointersMap;
  std::map<int, std::shared_ptr<Lidar>> lidarsPointersMap;

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
  const std::string jsonFilePath = argv[1];

  // Read and parse the JSON file
  Json::Value root;
  Json::Reader reader;
  std::ifstream jsonFile(("/home/lattinone/RobotProgramming/Project/workspace/src/mrsim/test_data/" + jsonFilePath).c_str(), std::ifstream::binary); // We need a way to remove absolute path
  
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
  std::string mapFileName = root["map"].asString();

  // Create a World object
  World w;
  w.loadFromImage(("/home/lattinone/RobotProgramming/Project/workspace/src/mrsim/test_data/" + mapFileName).c_str()); // We need a way to remove absolute path

  // Create a shared pointer to the World
  std::shared_ptr<World> worldSharedPtr = std::make_shared<World>(w);

  /*
   * From here we should load the configuration of Robots and Lidars from the JSON
   * We access the root[items] config.
   * This is the base for all items in the world.
  */
  Json::Value items = root["items"];

  // Let's iterate over the whole items of the JSON and taking them one by one.
  for (unsigned int i = 0; i < items.size(); ++i) {
      const Json::Value& item = items[i];

      // Access item properties
      int id = item["id"].asInt();                            // ID
      std::string type = item["type"].asString();             // Type
      std::string frame_id = item["frame_id"].asString();     // Frame_ID
      std::string namespace_ = item["namespace"].asString();  // Namespace

      // Access the "pose" array
      const Json::Value& poseArray = item["pose"];
      double poseX = poseArray[0].asDouble();                 // Pose: X
      double poseY = poseArray[1].asDouble();                 // Pose: Y
      double poseTheta = poseArray[2].asDouble();             // Pose: Theta

      int parent = item["parent"].asInt();                    // Parent ID

      if(type == "robot") { // We are accessing a Robot
        double radius = item["radius"].asDouble();            // Radius
        double max_rv = item["max_rv"].asDouble();            // Maximum Rotation velocity
        double max_tv = item["max_tv"].asDouble();            // Maximum Translation velocity

        // Let's define the Robot Pose
        Pose robotPose = Pose();

        if(parent == -1) {  // It means that this Robot is a completely new Robot with no parent
          robotPose.translation = worldSharedPtr->grid2world(IntPoint(poseX, poseY));
          robotPose.theta = poseTheta;
          /**
           * If we pass directly a Robot object inside make_shared, we are trying to create a shared pointer from an already constructed object,
           * which is not how std::make_shared is intended to be used.
           * 
           * In fact, we are directly creating a std::shared_ptr<Robot> object and 
           * costructing a Robot object directly inside the std::make_shared function.
          */
          std::shared_ptr<Robot> robotSharedPtr = std::make_shared<Robot>(id, type, frame_id, namespace_, radius, worldSharedPtr, robotPose, max_rv, max_tv, parent);
          robotsPointersMap[id] = robotSharedPtr;
        } else {
          // If we want to place a Robot on top of another one, we must first get who's the parent and, eventually, its pose.
          std::shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMap[parent];
          //cout << "Pose in world of parent[" << parentRobotSharedPtr->id <<"]: " << parentRobotSharedPtr->pose_in_parent << endl;
          if (parentRobotSharedPtr == nullptr) { // Doesn't exists?
            cerr << "Watch out. The parent you specified for child with ID[" << id << "] doesn't exist. Check your JSON file."<< endl;
          }

          // Apply offsets if specified
          robotPose.translation = Point(poseX, poseY);
          robotPose.theta = poseTheta;

          // Now let's define the Robot which will sit on top of the parent
          std::shared_ptr<Robot> robotSharedPtr = std::make_shared<Robot>(id, type, frame_id, namespace_, radius, parentRobotSharedPtr, robotPose, max_rv, max_tv, parent);
          robotsPointersMap[id] = robotSharedPtr;
        }

      } else if (type == "lidar") { // We are accessing a Lidar
        double fov = item["fov"].asDouble();                     // FOV
        double max_range = item["max_range"].asDouble();         // Maximum range for beams
        double num_beams = item["num_beams"].asDouble();         // Maximum number of beams

        // Let's define the Lidar Pose
        Pose lidarPose = Pose();
        lidarPose.translation = worldSharedPtr->grid2world(IntPoint(poseX, poseY));
        lidarPose.theta = poseTheta;

        if(parent == -1) {  // It means that this Lidar is a completely new Lidar with no parent. It lives in the world.
          std::shared_ptr<Lidar> lidarSharedPtr = std::make_shared<Lidar>(id, type, frame_id, namespace_, fov, worldSharedPtr, lidarPose, max_range, num_beams, parent);
          lidarsPointersMap[id] = lidarSharedPtr;
        } else {
          // If we want to place a Lidar on top of another one, we must first get who's the parent and, eventually, its pose.
          // We assume that the Lidar is on top of the Robot. Kinda weird to have a Lidar on top of a Lidar, hence the std::shared_ptr<Robot>.
          std::shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMap[parent];

          if (parentRobotSharedPtr == nullptr) { // Doesn't exists?
            ROS_ERROR_STREAM("Watch out. The parent you specified for child with ID[" << id << "] doesn't exist. Check your JSON file.");
          }

          // Add the offsets if we specified them in the JSON
          if (poseX != 0 || poseY != 0) {
            lidarPose.translation = lidarPose.translation + Point(poseX, poseY);
          }

          // Now let's define the Robot which will sit on top of the parent
          std::shared_ptr<Lidar> lidarSharedPtr = std::make_shared<Lidar>(id, type, frame_id, namespace_, fov, parentRobotSharedPtr, lidarPose, max_range, num_beams, parent);
          lidarsPointersMap[id] = lidarSharedPtr;
        }        
      } else {
        ROS_ERROR_STREAM("What kind of type are your trying to add, bro? Accepted types are 'robot' and 'lidar'.");
        return 1;
      }
  }
 
  float delay = 0.07;
  int k;

  while (ros::ok()) {

    ros::spinOnce();

    // run a simulation iteration
    worldSharedPtr->timeTick(delay);
    worldSharedPtr->draw();

    k = cv::waitKeyEx(delay*1000)&255;
    switch (k) {
      case 81:  // Left arrow
        robotsPointersMap[0]->rv += 0.10;
        break;  
      case 82:  // Up arrow
        robotsPointersMap[0]->tv += 0.2;
        break;  
      case 83:  // Right arrow
        robotsPointersMap[0]->rv -= 0.10;
        break;  
      case 84:  // Down arrow
        robotsPointersMap[0]->tv -= 0.2;
        break;  
      case ' ':  // Spacebar
        robotsPointersMap[0]->tv = 0;
        robotsPointersMap[0]->rv = 0;
        robotsPointersMap[0]->pose_in_parent.theta = 0;
        break;
      case 27:  // Esc
        return 0;
      default:
        robotsPointersMap[0]->rv = 0;
        continue;
    }

    // visualize the simulation
    cv::waitKey(100);
  }

  return 0;
}
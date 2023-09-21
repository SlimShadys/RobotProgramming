#include "misc.h"
#include "robot.h"
#include "lidar.h"

#include <memory>
#include <ros/ros.h>

using namespace std;

void getRobotsAndLidars(shared_ptr<World> worldSharedPointer, Json::Value root)
{
  map<int, shared_ptr<Robot>> robotsPointersMap;
  map<int, shared_ptr<Lidar>> lidarsPointersMap;

  int robotsCounter = 0;
  int lidarsCounter = 0;

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
      string type = item["type"].asString();                  // Type
      string frame_id = item["frame_id"].asString();          // Frame_ID
      string namespace_ = item["namespace"].asString();       // Namespace

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
          robotPose.translation = worldSharedPointer->grid2world(IntPoint(poseX, poseY));
          robotPose.theta = poseTheta;
          /**
           * In the first version, we used make_shared to instantiate shared_ptr<Robot> objs dynamically and directly.
           * 
           * Since we are not in mrsim_node.cpp, if we use make_shared, the object will get destroyed within the lifespan of misc.cpp
           * Hence, we do not want them to be dynamic. We instantiate, then, a true object, then compute the shared_ptr<Robot> object.
           * 
           * Same thing happens for Lidars.
           * 
           * Thx ChatGPT :)
          */
          shared_ptr<Robot> robotSharedPtr(new Robot(id, type, frame_id, namespace_, radius, worldSharedPointer, robotPose, max_rv, max_tv, parent),
                                                [](Robot* r) {
                                                    // Custom cleanup actions here, if needed
                                                    //delete r; // Clean up the Robot object
                                                });
          robotsPointersMap[id] = robotSharedPtr;
        } else {
          // If we want to place a Robot on top of another one, we must first get who's the parent and, eventually, its pose.
          shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMap[parent];
          //cout << "Pose in world of parent[" << parentRobotSharedPtr->id <<"]: " << parentRobotSharedPtr->pose_in_parent << endl;
          if (parentRobotSharedPtr == nullptr) { // Doesn't exists?
            cerr << "Watch out. The parent you specified for child with ID[" << id << "] doesn't exist. Check your JSON file."<< endl;
          }

          // Apply offsets if specified
          robotPose.translation = Point(poseX, poseY);
          robotPose.theta = poseTheta;

          // Now let's define the Robot which will sit on top of the parent
          shared_ptr<Robot> robotSharedPtr(new Robot(id, type, frame_id, namespace_, radius, parentRobotSharedPtr, robotPose, max_rv, max_tv, parent),
                                      [](Robot* r) {
                                          // Custom cleanup actions here, if needed
                                          //delete r; // Clean up the Robot object
                                      });
          robotsPointersMap[id] = robotSharedPtr;
        }
        robotsCounter++;

      } else if (type == "lidar") { // We are accessing a Lidar
        double fov = item["fov"].asDouble();                     // FOV
        double max_range = item["max_range"].asDouble();         // Maximum range for beams
        double num_beams = item["num_beams"].asDouble();         // Maximum number of beams

        // Let's define the Lidar Pose
        Pose lidarPose = Pose();

        if(parent == -1) {  // It means that this Lidar is a completely new Lidar with no parent. It lives in the world.
          lidarPose.translation = worldSharedPointer->grid2world(IntPoint(poseX, poseY));
          lidarPose.theta = poseTheta;
          shared_ptr<Lidar> lidarSharedPtr(new Lidar(id, type, frame_id, namespace_, fov, worldSharedPointer, lidarPose, max_range, num_beams, parent),
                                                [](Lidar* l) {
                                                    // Custom cleanup actions here, if needed
                                                    //delete l; // Clean up the Lidar object
                                                });
          lidarsPointersMap[id] = lidarSharedPtr;
        } else {
          // If we want to place a Lidar on top of another one, we must first get who's the parent and, eventually, its pose.
          // We assume that the Lidar is on top of the Robot. Kinda weird to have a Lidar on top of a Lidar, hence the shared_ptr<Robot>.
          shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMap[parent];

          if (parentRobotSharedPtr == nullptr) { // Doesn't exists?
            ROS_ERROR_STREAM("Watch out. The parent you specified for child with ID[" << id << "] doesn't exist. Check your JSON file.");
          }

          // Apply offsets if specified
          lidarPose.translation = Point(poseX, poseY);
          lidarPose.theta = poseTheta;

          // Now let's define the Robot which will sit on top of the parent
          shared_ptr<Lidar> lidarSharedPtr(new Lidar(id, type, frame_id, namespace_, fov, parentRobotSharedPtr, lidarPose, max_range, num_beams, parent),
                                      [](Lidar* l) {
                                          // Custom cleanup actions here, if needed
                                          //delete l; // Clean up the Lidar object
                                      });
          lidarsPointersMap[id] = lidarSharedPtr;
        }
        lidarsCounter++;
      } else {
        ROS_ERROR_STREAM("What kind of type are your trying to add, bro? Accepted types are 'robot' and 'lidar'.");
        return;
      }
  }

  cout << "Added " << robotsCounter << " robots to the world." << endl;
  cout << "Added " << lidarsCounter << " lidars to the world." << endl;
  return;
}

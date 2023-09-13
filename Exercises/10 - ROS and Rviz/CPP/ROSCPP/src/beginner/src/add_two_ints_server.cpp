#include "ros/ros.h"
#include "beginner/AddTwoInts.h"

/**
 * This function provides the service for adding two ints.
 * It takes in the request and response type defined in the srv file and returns a boolean.
 * 
 * Here the two ints (a, b) are added and stored in the response. 
 * Then some information about the request and response are logged. 
 * Finally the service returns true when it is complete.
 * 
 * */ 
bool add(beginner::AddTwoInts::Request &req, beginner::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("Getting request from [%s]: X = %ld, Y = %ld", req.identifier.c_str(), (long int)req.a, (long int)req.b);
  ROS_INFO("Sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  // Here the service is created and advertised over ROS.
  // Service name = add_two_ints
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
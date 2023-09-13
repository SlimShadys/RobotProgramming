#include "ros/ros.h"
#include "beginner/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 4)
  {
    ROS_INFO("Usage: add_two_ints_client identifier(str) X Y");
    return 1;
  }

  ros::NodeHandle n;
  /*
  * This creates a client for the add_two_ints service.
  * The ros::ServiceClient object is used to call the service later on.  
  */
  ros::ServiceClient client = n.serviceClient<beginner::AddTwoInts>("add_two_ints");
  beginner::AddTwoInts srv;
  srv.request.identifier = argv[1];
  srv.request.a = atoll(argv[2]);
  srv.request.b = atoll(argv[3]);

  if (client.call(srv)) // This actually calls the service.
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
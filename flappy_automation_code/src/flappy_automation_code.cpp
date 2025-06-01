#include <ros/ros.h>
#include "flappy_automation_code/flappy_controller.hpp"

int main(int argc, char **argv)
{
  //Initialization of nodehandle
  ros::init(argc, argv, "flappy_automation_code");
  ros::NodeHandle nh;
  BirdController bird_controller(nh);

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}

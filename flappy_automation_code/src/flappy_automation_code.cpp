#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

class BirdController
{
  private:
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd_;
    //Subscriber for velocity
    ros::Subscriber sub_vel_;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan_;

  public:
    BirdController(ros::NodeHandle &nh)
    {
      pub_acc_cmd_ = nh.advertise<geometry_msgs::Vector3>("/flappy_acc",1);
      sub_vel_ = nh.subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &BirdController::velCallback, this);
      sub_laser_scan_ = nh.subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &BirdController::laserScanCallback, this);
    }

    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
      // msg has the format of geometry_msgs::Vector3
      // Example of publishing acceleration command on velocity velCallback
      geometry_msgs::Vector3 acc_cmd;

      acc_cmd.x = 0;
      acc_cmd.y = 0;
      pub_acc_cmd_.publish(acc_cmd);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      //msg has the format of sensor_msgs::LaserScan
      //print laser angle and range
      ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
    }
};

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

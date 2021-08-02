#include "flappy_automation_code/flappy_controller.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

BirdController::BirdController(ros::NodeHandle &nh)
{
  pub_acc_cmd_ = nh.advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
  sub_vel_ = nh.subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &BirdController::velCallback, this);
  sub_laser_scan_ = nh.subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &BirdController::laserScanCallback, this);  

  broadcastBirdTransform(0.0f, 0.0f);
  resetPose();
}

void BirdController::resetPose()
{
  pose_.x = 0.0f;
  pose_.y = 0.0f;
  pose_.theta = 0.0f;
}

void BirdController::velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  geometry_msgs::Vector3 acc_cmd;

  acc_cmd.x = 0;
  acc_cmd.y = 0;
  pub_acc_cmd_.publish(acc_cmd);

  pose_.x += msg->x * T_DIFF;
  pose_.y += msg->y * T_DIFF;

  broadcastBirdTransform(pose_.x, pose_.y);
}

void BirdController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //msg has the format of sensor_msgs::LaserScan
  //print laser angle and range
  ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
}

void BirdController::broadcastBirdTransform(float x, float y)
{
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "laser_frame";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0f;
  transformStamped.transform.rotation.x = 0.0f;
  transformStamped.transform.rotation.y = 0.0f;
  transformStamped.transform.rotation.z = 0.0f;
  transformStamped.transform.rotation.w = 1.0f;

  br_.sendTransform(transformStamped);
}
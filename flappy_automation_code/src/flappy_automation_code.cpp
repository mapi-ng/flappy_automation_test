#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

const float T_DIFF = 1.0f/30.0f;

class BirdController
{
  private:
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd_;
    //Publisher for assembled pointcloud2
    ros::Publisher pub_pc2_;
    //Subscriber for velocity
    ros::Subscriber sub_vel_;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan_;

    tf2_ros::TransformBroadcaster br_;
    ros::ServiceClient client_;

  public:
    BirdController(ros::NodeHandle &nh)
    {
      ros::service::waitForService("assemble_scans2");
      client_ = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
      pub_acc_cmd_ = nh.advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
      pub_pc2_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2", 1);
      sub_vel_ = nh.subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &BirdController::velCallback, this);
      sub_laser_scan_ = nh.subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &BirdController::laserScanCallback, this);

      broadcastBirdTransform(0.0f, 0.0f);
    }

    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
      static float x = 0;
      static float y = 0;
      // msg has the format of geometry_msgs::Vector3
      // Example of publishing acceleration command on velocity velCallback
      geometry_msgs::Vector3 acc_cmd;

      acc_cmd.x = 0;
      acc_cmd.y = 0;
      pub_acc_cmd_.publish(acc_cmd);

      x += msg->x * T_DIFF;
      y += msg->y * T_DIFF;

      broadcastBirdTransform(x, y);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      laser_assembler::AssembleScans2 srv;
      srv.request.begin = ros::Time(0,0);
      srv.request.end = ros::Time::now();
      if (client_.call(srv))
      {
        size_t pointcloud_size = srv.response.cloud.data.size();
        ROS_INFO_STREAM("Got cloud with " << pointcloud_size << " points");
        if (pointcloud_size > 0)
          pub_pc2_.publish(srv.response.cloud);
      }
      else
      {
        ROS_INFO("Service call failed\n");
      }

      //msg has the format of sensor_msgs::LaserScan
      //print laser angle and range
      ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
    }

    void broadcastBirdTransform(float x, float y)
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

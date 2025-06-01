#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"

class BirdController
{
  private:
    geometry_msgs::Pose2D pose_;
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd_;
    //Subscriber for velocity
    ros::Subscriber sub_vel_;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan_;

    tf2_ros::TransformBroadcaster br_;

  public:
    static constexpr float T_DIFF = 1.0f/30.0f;

    BirdController() = delete;
    BirdController(ros::NodeHandle &nh);

    void resetPose();
    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void broadcastBirdTransform(float x, float y);
};

#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudProcessor
{
  private:
    bool is_started_ = false;
    bool publish_pointclouds_ = false;
    float leaf_size_ = 0.5f;
    ros::Rate rate_ = 4;

    ros::ServiceClient laser_assembler_client_;
    //Publisher for assembled pointcloud2
    ros::Publisher pub_pc2_;
    ros::Publisher pub_pc2_filtered_;

    std::unique_ptr<sensor_msgs::PointCloud2> requestPointCloud();
    void processPointClould(const sensor_msgs::PointCloud2& clould_msg);

  public:
    PointCloudProcessor() = delete;
    PointCloudProcessor(ros::NodeHandle &nh);

    void start() { is_started_ = true; }
    void stop() { is_started_ = false; }

    void spin();
};

#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudProcessor
{
  private:
    bool is_started_ = false;
    ros::Rate rate_ = 10;
    float leaf_size_ = 0.01;

    ros::ServiceClient laser_assembler_client_;
    //Publisher for assembled pointcloud2
    ros::Publisher pub_pc2_;

    std::unique_ptr<sensor_msgs::PointCloud2> requestPointCloud();
    void processPointClould(std::unique_ptr<sensor_msgs::PointCloud2> clould_msg);

  public:
    PointCloudProcessor() = delete;
    PointCloudProcessor(ros::NodeHandle &nh);

    void start() { is_started_ = true; }
    void stop() { is_started_ = false; }

    void spin();
};

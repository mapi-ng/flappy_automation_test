#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor
{
  private:
    bool is_started_ = false;
    ros::Rate rate_ = 10;
    float leaf_size_ = 0.01;

    ros::ServiceClient laser_assembler_client_;
    //Publisher for assembled pointcloud2
    ros::Publisher pub_pc2_;

    std::unique_ptr<sensor_msgs::PointCloud2> requestPointCloud()
    {
      laser_assembler::AssembleScans2 srv;
      srv.request.begin = ros::Time(0,0);
      srv.request.end = ros::Time::now();
      if (!laser_assembler_client_.call(srv))
      {
        throw "Failed to call laser_assembler service!";
      }

      size_t pointcloud_size = srv.response.cloud.data.size();
      ROS_INFO_STREAM("Got cloud with " << pointcloud_size << " points");
      if (pointcloud_size < 0)
      {
        throw "No pointcloud available!";
      }

      pub_pc2_.publish(srv.response.cloud);
      return std::make_unique<sensor_msgs::PointCloud2>(srv.response.cloud);
    }

    void processPointClould(std::unique_ptr<sensor_msgs::PointCloud2> clould_msg)
    {
      std::unique_ptr<pcl::PCLPointCloud2> cloud = std::make_unique<pcl::PCLPointCloud2>();
      std::unique_ptr<pcl::PCLPointCloud2> cloudFiltered = std::make_unique<pcl::PCLPointCloud2>();

      pcl_conversions::toPCL(*clould_msg, *cloud);
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud.get());

      // VoxelGrid downsampling filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloudPtr);
      sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      sor.filter(*cloudFiltered);
    }

  public:
    PointCloudProcessor() = delete;
    PointCloudProcessor(ros::NodeHandle &nh)
    {
      pub_pc2_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2", 1);
      ros::service::waitForService("assemble_scans2");
      laser_assembler_client_ = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    }

    void start() { is_started_ = true; }
    void stop() { is_started_ = false; }

    void spin()
    {
      while(ros::ok() && is_started_)
      {
        auto cloud_msg = requestPointCloud();

        ros::spinOnce();
        rate_.sleep();
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_processor");
  ros::NodeHandle nh;
  PointCloudProcessor pointcloud_processor(nh);

  pointcloud_processor.start();
  pointcloud_processor.spin();
  return 0;
}

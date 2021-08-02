#include "pointclould_processor/pointclould_processor.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle &nh)
{
  nh.getParam("leaf_size", leaf_size_);
  nh.getParam("publish_pointclouds", publish_pointclouds_);
  ROS_INFO_STREAM("leaf_size set to: " << leaf_size_);
  ROS_INFO_STREAM("publish_pointclouds set to: " << publish_pointclouds_);
  pub_pc2_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2", 1);
  pub_pc2_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2_filtered", 1);
  ros::service::waitForService("/assemble_scans2");
  laser_assembler_client_ = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");
}

std::unique_ptr<sensor_msgs::PointCloud2> PointCloudProcessor::requestPointCloud()
{
  laser_assembler::AssembleScans2 srv;
  srv.request.begin = ros::Time(0,0);
  srv.request.end = ros::Time::now();
  if (!laser_assembler_client_.call(srv))
  {
    throw std::runtime_error("Failed to call laser_assembler service!");
  }

  size_t pointcloud_size = srv.response.cloud.data.size();
  ROS_INFO_STREAM("Got cloud with " << pointcloud_size << " points");
  if (pointcloud_size < 0)
  {
    throw std::runtime_error("No pointcloud available!");
  }

  if (publish_pointclouds_)
    pub_pc2_.publish(srv.response.cloud);

  return std::make_unique<sensor_msgs::PointCloud2>(srv.response.cloud);
}

void PointCloudProcessor::processPointClould(const sensor_msgs::PointCloud2& clould_msg)
{
  // Raw pointer to work with PCL interfaces
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;

  std::unique_ptr<pcl::PCLPointCloud2> cloudFiltered = std::make_unique<pcl::PCLPointCloud2>();
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  
  pcl_conversions::toPCL(clould_msg, *cloud);

  // VoxelGrid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  sor.filter(*cloudFiltered);

  if (publish_pointclouds_)
  {
    sensor_msgs::PointCloud2 filtered_pc;
    pcl_conversions::fromPCL(*cloudFiltered, filtered_pc);
    pub_pc2_filtered_.publish(filtered_pc);
    ROS_INFO_STREAM("Filtered pointcloud size: " << filtered_pc.data.size());
  }
}

void PointCloudProcessor::spin()
{
  while(ros::ok() && is_started_)
  {
    try
    {
      auto cloud_msg = requestPointCloud();
      processPointClould(*cloud_msg);
    }
    catch (std::runtime_error ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }

    ros::spinOnce();
    rate_.sleep();
  }
}
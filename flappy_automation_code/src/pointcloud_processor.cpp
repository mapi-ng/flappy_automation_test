#include "pointclould_processor/pointclould_processor.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle &nh)
{
  nh.getParam("leaf_size", leaf_size_);
  nh.getParam("publish_pointclouds", publish_pointclouds_);
  nh.getParam("cluster_tolerance", cluster_tolerance_);
  nh.getParam("min_cluster_size", min_cluster_size_);
  nh.getParam("max_cluster_size", max_cluster_size_);
  nh.getParam("pointcloud_acquisition_time", pointcloud_acquisition_time_);

  ROS_INFO_STREAM("leaf_size set to: " << leaf_size_);
  ROS_INFO_STREAM("publish_pointclouds set to: " << publish_pointclouds_);
  ROS_INFO_STREAM("cluster_tolerance set to: " << cluster_tolerance_);
  ROS_INFO_STREAM("min_cluster_size set to: " << min_cluster_size_);
  ROS_INFO_STREAM("max_cluster_size set to: " << max_cluster_size_);
  ROS_INFO_STREAM("pointcloud_acquisition_time set to: " << pointcloud_acquisition_time_);

  pub_pc2_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2", 1);
  pub_pc2_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/flappy_pointcloud2_filtered", 1);
  ros::service::waitForService("/assemble_scans2");
  laser_assembler_client_ = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");
}

std::unique_ptr<sensor_msgs::PointCloud2> PointCloudProcessor::requestPointCloud()
{
  laser_assembler::AssembleScans2 srv;
  srv.request.begin =
    ros::Time::now() - ros::Duration(pointcloud_acquisition_time_, 0);
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

  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
  
  pcl_conversions::toPCL(clould_msg, *cloud);

  // VoxelGrid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_ptr);
  sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  sor.filter(cloud_filtered);

  if (publish_pointclouds_)
  {
    sensor_msgs::PointCloud2 filtered_pc;
    pcl_conversions::fromPCL(cloud_filtered, filtered_pc);
    pub_pc2_filtered_.publish(filtered_pc);
    ROS_DEBUG_STREAM("Filtered pointcloud size: " << filtered_pc.data.size());
  }

  auto clusters = extractClusters(cloud_filtered);

  /* TODO:
   * 1. Bounding box for each cluster
   * 2. Size of gaps in Y axis between each two neighboring clusters.
   * 3. Filter out only gaps big enough for bird to pass.
   * 4. Return position of the gap closest to the Y coordinate of the bird.
   */
}

PointCloudXYZAlignedVector PointCloudProcessor::extractClusters(const pcl::PCLPointCloud2& cloud)
{
  // Raw pointer to work with PCL interfaces
  PointCloudXYZ *cloud_xyz(new PointCloudXYZ);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(cloud, *cloud_xyz);
  PointCloudXYZ::ConstPtr cloud_xyz_ptr(cloud_xyz);

  std::vector<pcl::PointIndices> cluster_indices;

  search_tree->setInputCloud(cloud_xyz_ptr);

  ROS_INFO("Segmenting to clusters...");
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(search_tree);
  ec.setInputCloud(cloud_xyz_ptr);
  ec.extract(cluster_indices);
  ROS_INFO("Done");

  ROS_INFO_STREAM("Num indices: " << cluster_indices.size());

  // aligned_allocator is required when using STL containers before C++17
  // http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html#title1
  PointCloudXYZAlignedVector clustered_clouds;

  for (auto cluster_idx : cluster_indices)
  {
    PointCloudXYZ cluster;
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(cluster_idx));
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_xyz_ptr);
    extract.setIndices(indices_ptr);
    extract.setNegative(false);
    extract.filter(cluster);
    clustered_clouds.push_back(cluster);
  }

  ROS_INFO_STREAM("Num clusters: " << clustered_clouds.size());

  return clustered_clouds;
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

#include "pointclould_processor/pointclould_processor.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_processor");
  ros::NodeHandle nh("~");
  PointCloudProcessor pointcloud_processor(nh);

  pointcloud_processor.start();
  pointcloud_processor.spin();
  return 0;
}

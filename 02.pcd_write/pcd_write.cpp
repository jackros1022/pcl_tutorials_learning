#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// pcl/io/pcd_io.h is the header that contains the definitions for PCD I/O operations
// pcl/point_types.h contains definitions for several PointT type structures (pcl::PointXYZ in our case).

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
    // struct PointXYZ
    // {
    //   float x;
    //   float y;
    //   float z;
    // };

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height); //求出大小以便，填充数据

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud); // 保存位ASCII形式

  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  //打印出来数据
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
/*!
 * \file measure_objects_rgb.cpp
 * \brief Given one or more .pcd files containing PointXYZRGB points,\
 * 	this program measures the center location and size of each and\
 *  prints the results to the console.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 18, 2012
 */

#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "rail_pcl_object_segmentation/pcl_measurement.hpp"

/*!
 * \brief Main function for measure_objects_rgb executable.
 * 
 * Reads each specified .pcd file, and then measures the average center
 * of the point cloud and the bounding sphere's radius for the point 
 * cloud. Each of these are printed to the console.
 */
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: measure_objects path/to/object0.pcd ..." << std::endl;
    std::cerr << " Input files must contain points of type PointXYZRGB." << std::endl;
    std::cerr << " Each point cloud's average center point and minimum" << std::endl
        << " bounding sphere's radius will be computed and printed to the console." << std::endl;
    return -1;
  }

  pcl::PCDReader reader;

  // Loop across all input paths
  for (int i = 1; i < argc; i++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    std::string fileName(argv[i]);

    // Load
    std::cout << "Reading " << fileName << "..." << std::endl;
    reader.read(fileName, *cloud);
    std::cout << fileName << " has " << cloud->size() << " points." << std::endl;

    // Measure
    pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(cloud);
    double sphereRadius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center);
    double averageHue = rail::ComputePointCloudAverageHue(cloud);

    std::cout << " Center (x,y,z): " << center << std::endl;
    std::cout << " Bounding sphere radius: " << sphereRadius << std::endl;
    std::cout << " Average hue: " << averageHue << std::endl;
  }

  return 0;
}

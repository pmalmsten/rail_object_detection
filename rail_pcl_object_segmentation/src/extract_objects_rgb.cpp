/*!
 * \file extract_objects_rgb.cpp
 * \brief Given a .pcd file containing PointXYZRGB points, this program\
 *  extracts all objects and saves them as separate object_*.pcd files\
 *  in the working directory. 
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 18, 2012
 */

#include <sstream>
#include <string>
#include <cstring>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"

/*!
 * \brief Main function for extract_objects_rgb executable.
 * 
 * Reads the specified .pcd file, extracts all objects from it, and
 * saves them as separate .pcd files in the current working directory.
 * 
 * Each created file is called object_I_rgb.pcd, where I is an integer
 * which increases from 0.
 */
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: extract_objects_rgb [options] path/to/input.pcd" << std::endl;
    std::cerr << " Options:" << std::endl;
    std::cerr << "   --debug            Saves all intermediate point clouds to disk, including" << std::endl;
    std::cerr << "                      the original minus planes and each extracted plane" << std::endl;
    std::cerr << " Input file must contain points of type PointXYZRGB." << std::endl;
    std::cerr << " Extracted objects are saved to object_*.pcd in the" << std::endl << " current directory."
        << std::endl;
    return 1;
  }
  
  bool inDebugMode = (argc == 3 && strcmp(argv[1], "--debug") == 0);
  pcl::PCDWriter writer;

  // Read input file
  std::cout << "Reading '" << argv[argc - 1] << "'..." << std::endl;
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  reader.read(argv[argc - 1], *cloud);

  // Down sample
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);
  
  if(inDebugMode) {
      std::cout << "Writing downsampled cloud as 'input_downsampled_rgb.pcd'" << std::endl;
      writer.write("input_downsampled_rgb.pcd", *processed_cloud);
  }

  // Remove planes
  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);
  
  if(inDebugMode) {
      // Dump original w/o planes
      std::cout << "Writing input without large planes as 'input_minus_planes_rgb.pcd'" << std::endl;
      writer.write("input_minus_planes_rgb.pcd", *processed_cloud);
      
      // Dump planes
      unsigned int i = 0;
      for(std::vector<rail::DiscoveredPlanePtr>::iterator it = planes.begin(); it != planes.end(); ++it) {
          std::stringstream fileName;
          fileName << "extracted_plane_" << i++ << "_rgb.pcd";
          
          std::cout << " Writing extracted plane to " << fileName.str() << std::endl;
          writer.write(fileName.str(), (*(*it)).planeCloud);
      }
  }

  // Extract objects
  std::cout << "Extracting objects..." << std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  std::cout << " Extracted " << objects.size() << " objects." << std::endl;

  // Save results
  int i = 0;
  for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = objects.begin(); it != objects.end(); ++it)
  {
    std::stringstream fileName;
    fileName << "object_" << i << "_rgb.pcd";

    std::cout << "Writing " << fileName.str() << std::endl;

    writer.write(fileName.str(), *(*it));
    i++;
  }

  return 0;
}

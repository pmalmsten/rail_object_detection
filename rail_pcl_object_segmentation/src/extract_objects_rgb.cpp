/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Worcester Polytechnic Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Worcester Polytechnic Institute nor the 
 *     names of its contributors may be used to endorse or promote 
 *     products derived from this software without specific prior 
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *   Author: Paul Malmsten
 *  Version: Sep 20, 2012
 *
 *********************************************************************/

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
    std::cerr << "Usage: extract_objects path/to/input.pcd" << std::endl;
    std::cerr << " Input file must contain points of type PointXYZRGB." << std::endl;
    std::cerr << " Extracted objects are saved to object_*.pcd in the" << std::endl << " current directory."
        << std::endl;
    return -1;
  }

  // Read input file
  std::cout << "Reading '" << argv[1] << "'..." << std::endl;
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  reader.read(argv[1], *cloud);

  // Process
  std::cout << "Extracting objects..." << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  std::cout << " Extracted " << objects.size() << " objects." << std::endl;

  // Save results
  pcl::PCDWriter writer;
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

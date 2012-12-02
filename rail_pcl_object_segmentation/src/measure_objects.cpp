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
 *  Version: Sep 18, 2012
 *
 *********************************************************************/

/*!
 * \file measure_objects.cpp
 * \brief Given one or more .pcd files, this program measures\
 *  the center location and size of each and prints the results to the\
 *  console.
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
 * \brief Main function for measure_objects executable.
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
    std::cerr << " Each point cloud's average center point and minimum" << std::endl
        << " bounding sphere's radius will be computed and printed to the console." << std::endl;
    return -1;
  }

  pcl::PCDReader reader;

  // Loop across all input paths
  for (int i = 1; i < argc; i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::string fileName(argv[i]);

    // Load
    std::cout << "Reading " << fileName << "..." << std::endl;
    reader.read(fileName, *cloud);
    std::cout << fileName << " has " << cloud->size() << " points." << std::endl;

    // Measure
    pcl::PointXYZ center = rail::AveragePointCloudToPoint<pcl::PointXYZ>(cloud);
    double sphereRadius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZ>(cloud, center);

    std::cout << " Center (x,y,z): " << center << std::endl;
    std::cout << " Bounding sphere radius: " << sphereRadius << std::endl;
  }

  return 0;
}

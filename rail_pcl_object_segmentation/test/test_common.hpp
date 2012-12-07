/*!
 * \file test_helpers.cpp
 * \brief Defines common routines used by the automated tests.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 17, 2012
 */

#ifndef _TEST_COMMON_H
#define _TEST_COMMON_H

#include <pcl/point_types.h>

/*!
 * \brief Loads a point cloud from a .pcd file
 * 
 * Loads a PCL point cloud from a .pcd file and returns a pointer to
 * the point cloud.
 */
template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr LoadPointCloudFromFile(const char* filePath)
  {
    pcl::PCDReader reader;
    typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
    reader.read(filePath, *cloud);
    return cloud;
  }

#endif

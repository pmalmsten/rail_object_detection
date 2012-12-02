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
 *  Version: Nov 27, 2012
 *
 *********************************************************************/

/*!
 * \file extract_objects_srv.cpp
 * \brief Provides a ROS service for extracting objects and surfaces from a given point cloud.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Nov 27, 2012
 */

#include "ros/ros.h"
#include "rail_pcl_object_segmentation/extract_objects.h"
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"
#include <vector>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"

struct object_filter : public std::unary_function<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool>
{
  double min_distance;
  double max_distance;
  double min_radius;
  double max_radius;

  // Filter function
  bool operator()(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const
  {
    pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(cloud);
    double radius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center);
    ROS_DEBUG("Filter visiting point cloud:");

    if (min_distance >= 0 || max_distance >= 0)
    {
      // Compute center point distance from 0,0
      double distance = sqrt(pow(center.x, 2.0) + pow(center.y, 2.0) + pow(center.z, 2.0));
      ROS_DEBUG_STREAM(" Point cloud distance from origin (m): " << distance);

      if (min_distance >= 0 && distance < min_distance)
      {
        ROS_DEBUG("  Distance too small; discarding object");
        return false;
      }

      if (max_distance >= 0 && distance > max_distance)
      {
        ROS_DEBUG("  Distance too large; discarding object");
        return false;
      }
    }

    ROS_DEBUG_STREAM(" Point cloud spherical radius (m): " << radius);
    if (min_radius >= 0 && radius < min_radius)
    {
      ROS_DEBUG("  Point cloud radius too small; discarding object");
      return false;
    }
    if (max_radius >= 0 && radius > max_radius)
    {
      ROS_DEBUG("  Point cloud radius too large; discarding object");
      return false;
    }

    ROS_DEBUG(" Point cloud OK");
    return true;
  }
};

bool extract(rail_pcl_object_segmentation::extract_objects::Request &req,
             rail_pcl_object_segmentation::extract_objects::Response &res)
{
  //convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(req.cloud, *pclCloud);

  //extract point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(pclCloud, processed_cloud);

  // Extract planes
  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  // Filter planes which do not appear flat
  if (req.plane_slope_tolerance > 0)
    rail::FilterInclinedPlanes(planes, req.plane_slope_tolerance);

  // Store extracted planes in response
  for (std::vector<rail::DiscoveredPlanePtr>::iterator it = planes.begin(); it != planes.end(); ++it)
  {
    // Pass DiscoveredPlane by value
    res.planes.push_back(*(*it));
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter point clouds
  struct object_filter correct_params;
  correct_params.min_distance = req.constraints.object_min_sensor_range;
  correct_params.max_distance = req.constraints.object_max_sensor_range;
  correct_params.min_radius = req.constraints.object_min_spherical_radius;
  correct_params.max_radius = req.constraints.object_max_spherical_radius;

  ROS_DEBUG("Filtering parameters: ");
  ROS_DEBUG_STREAM(" Object min distance: " << correct_params.min_distance);
  ROS_DEBUG_STREAM(" Object max distance: " << correct_params.max_distance);
  ROS_DEBUG_STREAM(" Object min radius: " << correct_params.min_radius);
  ROS_DEBUG_STREAM(" Object max radius: " << correct_params.max_radius);
  ROS_DEBUG_STREAM(" Plane slope tolerance: " << req.plane_slope_tolerance);

  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(correct_params)), objects.end());

  //convert back to point clouds
  sensor_msgs::PointCloud2 object;

  for (int i = 0; i < (int)objects.size(); i++)
  {
    const pcl::PointCloud<pcl::PointXYZRGB> tempCloud = *objects.at(i);
    pcl::toROSMsg(tempCloud, object);
    res.clouds.push_back(object);
  }

  ROS_INFO_STREAM("Returning " << objects.size() << " objects");
  ROS_INFO_STREAM("Returning " << planes.size() << " surface(s)");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_objects_srv");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("extract_objects", extract);
  ROS_INFO("Ready to extract objects");
  ros::spin();

  return 0;
}


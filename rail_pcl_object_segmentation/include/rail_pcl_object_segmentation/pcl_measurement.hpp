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
 *  Version: Sep 21, 2012
 *
 *********************************************************************/

/*!
 * \file test_helpers.cpp
 * \brief Defines common point cloud measurement routines.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 21, 2012
 */

#ifndef _PCL_MEASUREMENT_H
#define _PCL_MEASUREMENT_H

#include <pcl/point_types.h>
#include <cmath>
#include <iostream>

#include "rail_pcl_object_segmentation/common.hpp"

namespace rail
{
/*!
 * \brief Averages together the points in a point cloud and returns the resulting point.
 * 
 * \param cloud The point cloud to average.
 * \return The averaged point within the cloud.
 */
template<typename PointT>
  PointT AveragePointCloudToPoint(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    PointT centerPoint;

    // Sum all points together
    for (typename pcl::PointCloud<PointT>::iterator cloud_point = cloud->begin(); cloud_point != cloud->end();
        ++cloud_point)
    {
      centerPoint.x += (*cloud_point).x;
      centerPoint.y += (*cloud_point).y;
      centerPoint.z += (*cloud_point).z;
    }

    // Divide by point count
    centerPoint.x /= cloud->size();
    centerPoint.y /= cloud->size();
    centerPoint.z /= cloud->size();

    return centerPoint;
  }

/*!
 * \brief Determines the bounding radius of the given point cloud from the given point. 
 * 
 * This function computes the radius of a bounding sphere for all
 * of the points in the given cloud. The bounding sphere is centered
 * at the given center point, and its radius is the minimum radius
 * such that the sphere contains all of the points in the given
 * point cloud.
 * 
 * \param cloud The point cloud measure.
 * \param center The point at which the center of the bounding sphere should be located.
 * \return The radius of the bounding sphere.
 */
template<typename PointT>
  double ComputePointCloudBoundingRadiusFromPoint(typename pcl::PointCloud<PointT>::Ptr cloud, PointT center)
  {
    double boundingRadius = 0;

    for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      PointT currentPoint = *it;

      // Measure distance
      double radius = sqrt(
          pow(currentPoint.x - center.x, 2) + pow(currentPoint.y - center.y, 2) + pow(currentPoint.z - center.z, 2));

      if (radius > boundingRadius)
      {
        boundingRadius = radius;
      }
    }

    return boundingRadius;
  }

#define CUP_MAX_RADIUS 0.15
#define CUP_MIN_RADIUS 0.01
/*!
 * \brief Determines whether the given point cloud is the approximate size of a cup.
 */
template<typename PointT>
  struct is_cup_size : public std::unary_function<typename pcl::PointCloud<PointT>::Ptr, bool>
  {
    bool operator()(const typename pcl::PointCloud<PointT>::Ptr cloud) const
    {
      PointT center = rail::AveragePointCloudToPoint<PointT>(cloud);
      double radius = rail::ComputePointCloudBoundingRadiusFromPoint<PointT>(cloud, center);
      return radius > CUP_MIN_RADIUS && radius < CUP_MAX_RADIUS;
    }
  };

/*!
 * \brief Determines whether the given point cloud close enough to the origin.
 */
template<typename PointT>
  struct is_closer_than : public std::unary_function<typename pcl::PointCloud<PointT>::Ptr, bool>
  {
    double max_distance;
    bool operator()(const typename pcl::PointCloud<PointT>::Ptr cloud) const
    {
      PointT center = rail::AveragePointCloudToPoint<PointT>(cloud);
      double distance = sqrt(pow(center.x, 2.0) + pow(center.y, 2.0) + pow(center.z, 2.0));
      return distance < max_distance;
    }
  };

/*!
 * \brief Determines whether the given plane is level within a given tolerance.
 * 
 * Given a plane equation in the form ax + by + cz + d = 0 in a DiscoveredPlane
 * structure, this function evaluates whether it is level within a given
 * tolerance.
 * 
 * This is measured by taking the partial derivative of the plane
 * with respect to x and y. These become:
 * 
 * df/dx: a = 0
 * df/dy: b = 0
 * 
 * Intuitively, a defines the slope of the plane along the x axis,
 * and b defines the slope of the plane along the y axis.
 * 
 * If either of these exceeds a given tolerance, then it is declared
 * that the plane is not level, and false is returned. Otherwise,
 * true is returned.
 */
struct is_level : public std::unary_function<rail::DiscoveredPlanePtr, bool>
{
  double max_slope;
  bool operator()(const rail::DiscoveredPlanePtr plane) const
  {
    if (fabs(plane->a) > max_slope)
      return false;

    if (fabs(plane->b) > max_slope)
      return false;

    return true;
  }
};

/**
 * 
 * 
 * \param [inout] planes The vector of planes to filter in-place.
 * \param [in] max_slope The maximum slope along the x or y axis that
 *  a plane must not exceed.
 */
void FilterInclinedPlanes(std::vector<rail::DiscoveredPlanePtr>& planes, double max_slope)
{
  struct is_level is_plane_level;
  is_plane_level.max_slope = max_slope;

  // Remove non-level planes
  planes.erase(std::remove_if(planes.begin(), planes.end(), std::not1(is_plane_level)), planes.end());
}
}

#endif

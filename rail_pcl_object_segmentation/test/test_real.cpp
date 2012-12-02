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
 *  Version: Sep 17, 2012
 *
 *********************************************************************/

/*!
 * \file test_real.cpp
 * \brief Tests functionality of pcl_segmenter.hpp against real point\
 *  clouds taken from a kinect.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 17, 2012
 */

#include <algorithm>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"
#include "gtest/gtest.h"

#include "test_common.hpp"

/*!
 * \brief No objects should be detected on an empty floor.
 */TEST(RealTests, EmptyFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/empty_floor_near_wall_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  EXPECT_EQ(0, objects.size());

}

/*!
 * \brief One cup on an empty floor should be detected.
 */TEST(RealTests, OneCupOnFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/1_cup_on_floor_near_wall_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  EXPECT_EQ(1, objects.size());

}

/*!
 * \brief Three cup on an empty floor should be detected.
 */TEST(RealTests, ThreeCupsOnFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/3_cups_on_floor_near_wall_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter out objects larger than a cup
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(rail::is_cup_size<pcl::PointXYZRGB>())),
                objects.end());

  EXPECT_EQ(3, objects.size());

}

/*!
 * \brief One cup on a table should be detected.
 */TEST(RealTests, OneCupOnTableNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/1_cup_on_table_near_wall_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter out objects larger than a cup
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(rail::is_cup_size<pcl::PointXYZRGB>())),
                objects.end());

  EXPECT_EQ(1, objects.size());

}

/*!
 * \brief Three cups on a table should be detected.
 */TEST(RealTests, ThreeCupsOnTableNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/3_cups_on_table_near_wall_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter out objects larger than a cup
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(rail::is_cup_size<pcl::PointXYZRGB>())),
                objects.end());

  // Filter out objects that are too far away
  rail::is_closer_than<pcl::PointXYZRGB> is_close_enough;
  is_close_enough.max_distance = 1.20;
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(is_close_enough)), objects.end());

  EXPECT_EQ(3, objects.size());
  // 3 cups, 2 table legs

}

/*!
 * \brief A yellow block in the downstairs interaction lab should be detected.
 */TEST(RealTests, YellowBlockOnFloorInLabRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/yellow_block_on_floor_real_rgb.pcd");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter out objects larger than a cup
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(rail::is_cup_size<pcl::PointXYZRGB>())),
                objects.end());

  // Filter out objects that are too far away
  rail::is_closer_than<pcl::PointXYZRGB> is_close_enough;
  is_close_enough.max_distance = 1.10;
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(is_close_enough)), objects.end());

  EXPECT_EQ(1, objects.size());

}

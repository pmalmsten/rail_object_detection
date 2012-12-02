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
 * \file test_sim.cpp
 * \brief Tests functionality of pcl_segmenter.cpp
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 17, 2012
 */

#include <algorithm>
#include <pcl/io/pcd_io.h>

#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"
#include "gtest/gtest.h"

#include "test_common.hpp"

/*! Tests that no objects are detected in an empty area.
 */TEST(SimTests, EmptyFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/empty_floor_near_wall_sim_rgb.pcd");
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

  EXPECT_EQ(0, objects.size());

  // Filter out sharply inclined planes
  rail::FilterInclinedPlanes(planes, 0.4);

  // Expect floor
  EXPECT_EQ(1, planes.size());
}

/*!
 * Tests that a cup on the floor near a wall with no other objects nearby
 * is detected appropriately.
 */TEST(SimTests, CupOnFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/1_cup_on_floor_near_wall_sim_rgb.pcd");
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
 * Tests that all point clouds have the same frame id as their original
 */TEST(SimTests, CupOnFloorNearWallRGBFrameIdPersisted)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/1_cup_on_floor_near_wall_sim_rgb.pcd");
  cloud->header.frame_id = "jbop";
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
  EXPECT_EQ("jbop", objects.at(0)->header.frame_id);
}

/*!
 * Tests that 3 cups on the floor near a wall with no other objects nearby
 * are detected appropriately.
 */TEST(SimTests, ThreeCupsOnFloorNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/3_cups_on_floor_near_wall_sim_rgb.pcd");
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

TEST(SimTests, EmptyTableNearWall)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZ>(
      "test/fixtures/empty_table_near_wall_sim.pcd");
  pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());

  rail::DownsampleCloud<pcl::PointXYZ>(cloud, processed_cloud);

  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZ>(processed_cloud, processed_cloud, planes);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZ>(processed_cloud, objects);

  // Filter out objects larger than a cup
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(rail::is_cup_size<pcl::PointXYZ>())),
                objects.end());

  // Filter out objects that are too far away
  rail::is_closer_than<pcl::PointXYZ> is_close_enough;
  is_close_enough.max_distance = 1.10;
  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(is_close_enough)), objects.end());

  EXPECT_EQ(0, objects.size());

}

/*!
 * Tests that a blue cup on a white table surface result in two objects
 * being discovered.
 */TEST(SimTests, OneCupOnTableNearWallRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadPointCloudFromFile<pcl::PointXYZRGB>(
      "test/fixtures/1_cup_on_table_sim_xyzrgb.pcd");
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

  if (objects.size() > 0)
  {
    pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(objects.at(0));
    ASSERT_NEAR(-0.0412, center.x, 0.005);
    ASSERT_NEAR(-0.1320, center.y, 0.02);
    ASSERT_NEAR(0.9414, center.z, 0.02);
  }

}

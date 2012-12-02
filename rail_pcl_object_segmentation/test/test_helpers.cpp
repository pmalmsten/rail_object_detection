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
 * \file test_helpers.cpp
 * \brief Tests functionality of helper functions defined in pcl_segmentation.cpp
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 17, 2012
 */

#include "rail_pcl_object_segmentation/pcl_measurement.hpp"
#include "gtest/gtest.h"

TEST(TestHelpers, TestAveragePointCloudToPoint)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ p1, p2, p3, p4;

  // Set 4 points to be exactly 1 unit away from 1,1,1
  p1.x = 2;
  p1.y = 1;
  p1.z = 1;

  p2.x = 0;
  p2.y = 1;
  p2.z = 1;

  p3.x = 1;
  p3.y = 2;
  p3.z = 1;

  p4.x = 1;
  p4.y = 0;
  p4.z = 1;

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  pcl::PointXYZ center = rail::AveragePointCloudToPoint<pcl::PointXYZ>(cloud);

  // Expect a center of 1,1,1
  ASSERT_NEAR(1, center.x, 0.00001);
  ASSERT_NEAR(1, center.y, 0.00001);
  ASSERT_NEAR(1, center.z, 0.00001);
}

TEST(TestHelpers, TestComputePointCloudBoundingRadiusFromPointAtOrigin)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ center, p1, p2, p3, p4;

  // Center at origin
  center.x = 0;
  center.y = 0;
  center.z = 0;

  p1.x = 3;
  p1.y = 3;
  p1.z = 0;
  // Distance: sqrt(3^2 + 3^2) = sqrt(18)

  p2.x = -5;
  p2.y = 10;
  p2.z = 7;
  // Distance: sqrt((-5)^2 + 10^2 + 7^2) = sqrt(174)

  p3.x = 7;
  p3.y = -7;
  p3.z = 2;
  // Distance: sqrt(7^2 + (-7)^2 + 2^2) = sqrt(102)

  p4.x = 10;
  p4.y = -10;
  p4.z = 1;
  // Distance: sqrt(10^2 + (-10)^2 + 1^2) = sqrt(201)  <-- Max

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  ASSERT_NEAR(14.1774, rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZ>(cloud, center), 0.0001);
}

TEST(TestHelpers, TestComputePointCloudBoundingRadiusFromPointAtOneOneOne)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ center, p1, p2, p3, p4;

  // Center at 1,1,1 now
  center.x = 1;
  center.y = 1;
  center.z = 1;

  p1.x = 3;
  p1.y = 3;
  p1.z = 0;
  // Distance: sqrt((3-1)^2 + (3-1)^2) = sqrt(8)

  p2.x = -5;
  p2.y = 10;
  p2.z = 7;
  // Distance: sqrt((-5-1)^2 + (10-1)^2 + (7-1)^2) = sqrt(153)

  p3.x = 7;
  p3.y = -7;
  p3.z = 2;
  // Distance: sqrt((7-1)^2 + (-7-1)^2 + (2-1)^2) = sqrt(101)

  p4.x = 10;
  p4.y = -10;
  p4.z = 1;
  // Distance: sqrt((10-1)^2 + (-10-1)^2 + (1-1)^2) = sqrt(202)  <-- Max

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  ASSERT_NEAR(14.2126, rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZ>(cloud, center), 0.0001);
}

// PointXYZRGB tests

TEST(TestHelpers, TestAveragePointCloudToPointRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointXYZRGB p1, p2, p3, p4;

  // Set 4 points to be exactly 1 unit away from 1,1,1
  p1.x = 2;
  p1.y = 1;
  p1.z = 1;

  p2.x = 0;
  p2.y = 1;
  p2.z = 1;

  p3.x = 1;
  p3.y = 2;
  p3.z = 1;

  p4.x = 1;
  p4.y = 0;
  p4.z = 1;

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(cloud);

  // Expect a center of 1,1,1
  ASSERT_NEAR(1, center.x, 0.00001);
  ASSERT_NEAR(1, center.y, 0.00001);
  ASSERT_NEAR(1, center.z, 0.00001);
}

TEST(TestHelpers, TestComputePointCloudBoundingRadiusFromPointAtOriginRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointXYZRGB center, p1, p2, p3, p4;

  // Center at origin
  center.x = 0;
  center.y = 0;
  center.z = 0;

  p1.x = 3;
  p1.y = 3;
  p1.z = 0;
  // Distance: sqrt(3^2 + 3^2) = sqrt(18)

  p2.x = -5;
  p2.y = 10;
  p2.z = 7;
  // Distance: sqrt((-5)^2 + 10^2 + 7^2) = sqrt(174)

  p3.x = 7;
  p3.y = -7;
  p3.z = 2;
  // Distance: sqrt(7^2 + (-7)^2 + 2^2) = sqrt(102)

  p4.x = 10;
  p4.y = -10;
  p4.z = 1;
  // Distance: sqrt(10^2 + (-10)^2 + 1^2) = sqrt(201)  <-- Max

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  ASSERT_NEAR(14.1774, rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center), 0.0001);
}

TEST(TestHelpers, TestComputePointCloudBoundingRadiusFromPointAtOneOneOneRGB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointXYZRGB center, p1, p2, p3, p4;

  // Center at 1,1,1 now
  center.x = 1;
  center.y = 1;
  center.z = 1;

  p1.x = 3;
  p1.y = 3;
  p1.z = 0;
  // Distance: sqrt((3-1)^2 + (3-1)^2) = sqrt(8)

  p2.x = -5;
  p2.y = 10;
  p2.z = 7;
  // Distance: sqrt((-5-1)^2 + (10-1)^2 + (7-1)^2) = sqrt(153)

  p3.x = 7;
  p3.y = -7;
  p3.z = 2;
  // Distance: sqrt((7-1)^2 + (-7-1)^2 + (2-1)^2) = sqrt(101)

  p4.x = 10;
  p4.y = -10;
  p4.z = 1;
  // Distance: sqrt((10-1)^2 + (-10-1)^2 + (1-1)^2) = sqrt(202)  <-- Max

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->push_back(p4);

  ASSERT_NEAR(14.2126, rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center), 0.0001);
}

TEST(TestHelpers, TestRemoveInclinedPlanes)
{
  std::vector<rail::DiscoveredPlanePtr> planes;

  rail::DiscoveredPlanePtr plane1(new rail_pcl_object_segmentation::DiscoveredPlane());
  plane1->a = 0.3;
  plane1->b = 0.01;
  plane1->c = 1;
  plane1->c = 54;
  planes.push_back(plane1);

  rail::DiscoveredPlanePtr plane2(new rail_pcl_object_segmentation::DiscoveredPlane());
  plane2->a = 0.01;
  plane2->b = -0.3;
  plane2->c = 1;
  plane2->c = 54;
  planes.push_back(plane2);

  rail::DiscoveredPlanePtr plane3(new rail_pcl_object_segmentation::DiscoveredPlane());
  plane3->a = 0.3;
  plane3->b = 0.3;
  plane3->c = 1;
  plane3->c = 54;
  planes.push_back(plane3);

  rail::DiscoveredPlanePtr plane4(new rail_pcl_object_segmentation::DiscoveredPlane());
  plane4->a = 0.01;
  plane4->b = 0.01;
  plane4->c = 1;
  plane4->c = 54;
  planes.push_back(plane4);

  rail::FilterInclinedPlanes(planes, 0.1);

  EXPECT_EQ(1, planes.size());
}

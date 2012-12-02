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
 * \file update_environment.cpp
 * \brief Defines a node which implements the UpdateEnvironment service.
 * 
 * Upon calling this node's service, the following actions are taken:
 * 	1) All object points are filtered out of the static environment point cloud.
 *  2) All objects currently recognized by planning_environment are cleared.
 *  3) The planning_environment is informed of each object.
 *  4) The collision map is provided with a filtered static environment point cloud for processing.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Sep 21, 2012
 */

#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/io.h>
#include <pcl_ros/point_cloud.h>

#include "rail_object_discovery/NamedPointCloud2.h"
#include "rail_object_discovery/UpdateEnvironment.h"

ros::Publisher static_environment_publisher;
ros::Publisher collision_object_publisher;
ros::ServiceClient bounding_box_finder;

bool add_cloud_bounding_box_to_collision_environment(const rail_object_discovery::NamedPointCloud2& namedCloud)
{
  // Find 3D bounding box for object
  ROS_INFO_STREAM(" Calling service to compute bounding box...");
  object_manipulation_msgs::FindClusterBoundingBox2 boundingBoxRequest;
  boundingBoxRequest.request.cluster = namedCloud.cloud;

  if (!bounding_box_finder.call(boundingBoxRequest.request, boundingBoxRequest.response))
  {
    ROS_ERROR("Failed to call cluster bounding box service.");
    return false;
  }
  geometry_msgs::PoseStamped pose_stamped = boundingBoxRequest.response.pose;
  geometry_msgs::Vector3 dimensions = boundingBoxRequest.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box 2 3d client returned an error (0.0 bounding box)");
    return false;
  }

  ROS_INFO(" Bounding box determined, publishing object.");

  // Create add object message
  arm_navigation_msgs::CollisionObject objectMsg;
  objectMsg.id = namedCloud.name;
  objectMsg.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  objectMsg.header.stamp = ros::Time::now();
  objectMsg.header.frame_id = boundingBoxRequest.response.pose.header.frame_id;

  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = boundingBoxRequest.response.box_dims.x;
  object.dimensions[1] = boundingBoxRequest.response.box_dims.y;
  object.dimensions[2] = boundingBoxRequest.response.box_dims.z;
  objectMsg.shapes.push_back(object);
  objectMsg.poses.push_back(boundingBoxRequest.response.pose.pose);

  // Publish collision object
  collision_object_publisher.publish(objectMsg);

  return true;
}

/*!
 * \brief Handles calls to the 'update environment' service.
 */
bool update_environment_callback(rail_object_discovery::UpdateEnvironment::Request& request,
                                 rail_object_discovery::UpdateEnvironment::Response& response)
{

  /* Warning: causes segfault, at line 92 */
#if 0
  // Convert static environment point cloud to PCL
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr static_environment(new pcl::PointCloud< pcl::PointXYZRGB >());
  pcl::fromROSMsg(request.static_environment, *static_environment);

  ROS_INFO("Filtering object points out of point cloud...");
  // Filter object clouds out of point clouds
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  for(std::vector< sensor_msgs::PointCloud2 >::iterator it = request.objects.begin(); it != request.objects.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_environment_filtered(new pcl::PointCloud< pcl::PointXYZRGB >());

    // Convert static environment point cloud to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud< pcl::PointXYZRGB >());
    pcl::fromROSMsg(*it, *object);

    // Determine indices
    pcl::IndicesPtr common_indices(new std::vector< int >());
    pcl::getApproximateIndices<pcl::PointXYZRGB>(
        object,
        static_environment_filtered,
        *common_indices);

    // Filter
    extract.setInputCloud(static_environment);
    extract.setIndices(common_indices);
    extract.filter(*static_environment_filtered);

    // Update environment cloud to be filtered
    static_environment.swap(static_environment_filtered);
  }

  // Convert and publish result
  sensor_msgs::PointCloud2 filtered_static_environment;
  pcl::toROSMsg(*static_environment, filtered_static_environment);
  static_environment_publisher.publish(filtered_static_environment);
  ROS_INFO("Filtering complete; filtered cloud published.");
#endif

  // Clear all collision objects
  ROS_INFO("Clearing all existing objects in planning environment.");
  arm_navigation_msgs::CollisionObject clearAllObjectsMsg;
  clearAllObjectsMsg.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  clearAllObjectsMsg.id = "all";
  collision_object_publisher.publish(clearAllObjectsMsg);

  // Name each object and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.objects.size(); i++)
  {
    std::stringstream ss;
    ss << "object_" << i;

    // Return named objects
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.objects.at(i);
    namedCloud.name = ss.str();
    response.objects.push_back(namedCloud);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

    if (!add_cloud_bounding_box_to_collision_environment(namedCloud))
      return false;
  }

  // Name each surface and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.surfaces.size(); i++)
  {
    std::stringstream ss;
    ss << "surface_" << i;

    // Return named surfaces
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.surfaces.at(i);
    namedCloud.name = ss.str();
    response.surfaces.push_back(namedCloud);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

    if (!add_cloud_bounding_box_to_collision_environment(namedCloud))
      return false;
  }

  ROS_INFO("Service call complete.");
  return true;
}

/*!
 * \brief Main function for update environment service.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "update_environment_server");
  ros::NodeHandle priv("~");
  ros::NodeHandle n;

  // Read parameters
  std::string serviceName, boundingBoxServiceName, staticEnvironmentTopicName, collisionObjectTopicName;

  priv.param<std::string>("service_name", serviceName, "update_environment");
  priv.param<std::string>("bounding_box_service_name", boundingBoxServiceName, "find_cluster_bounding_box2");
  priv.param<std::string>("output/filtered_env_cloud_topic_name", staticEnvironmentTopicName,
                          "static_environment_filtered");
  priv.param<std::string>("output/collision_object_topic_name", collisionObjectTopicName, "collision_object");

  // Configure services and topics
  bounding_box_finder = n.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>(boundingBoxServiceName);
  ROS_INFO("Waiting for existence of bounding box service of name '%s'...", boundingBoxServiceName.c_str());
  bounding_box_finder.waitForExistence();

  static_environment_publisher = n.advertise<sensor_msgs::PointCloud2>(staticEnvironmentTopicName, 1000);
  ROS_INFO("Ready to publish filtered static collision point cloud at name '%s'", staticEnvironmentTopicName.c_str());
  collision_object_publisher = n.advertise<arm_navigation_msgs::CollisionObject>(collisionObjectTopicName, 1000);
  ROS_INFO("Ready to publish collision objects at name '%s'", collisionObjectTopicName.c_str());

  ros::ServiceServer service = n.advertiseService(serviceName, update_environment_callback);
  ROS_INFO("Update environment service available at name '%s'", serviceName.c_str());

  ros::spin();
  return 0;
}

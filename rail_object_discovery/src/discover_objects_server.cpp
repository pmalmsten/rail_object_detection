/*!
 * \file discover_objects_server.cpp
 * \brief Defines a node which implements the discover_objects service.
 *
 * Upon calling this node's service, the following actions are taken:
 *  1) A depth point cloud is captured from a Kinect.
 *  2) The point cloud is transformed into the base_link frame.
 *  3) Objects and planes are extracted from the point cloud
 *  4) (Optional) The UpdateEnvironment service is called with the discovered objects and surfaces.
 *  5) The resulting planes and surfaces are returned.
 *
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Nov 30, 2012
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_ros/transforms.h>

#include <rail_pcl_object_segmentation/extract_objects.h>
#include <rail_pcl_object_segmentation/ObjectConstraints.h>

#include "rail_object_discovery/discover_objects.h"
#include "rail_object_discovery/UpdateEnvironment.h"

#define TF_BUFFER_DURATION_SECS 10

std::string sensor_topic_name, target_frame_id, extract_objects_service_name, update_environment_service_name;
bool update_cloud_timestamp_if_too_old, should_update_enviroment;

tf::TransformListener* tf_listener;

bool discover_objects_callback(rail_object_discovery::discover_objects::Request &req,
                               rail_object_discovery::discover_objects::Response &res)
{
  ros::Duration ten_seconds(10.0);

  // Wait for and transform point cloud from sensor
  ROS_INFO("Waiting for point cloud...");
  sensor_msgs::PointCloud2 transformedCloud;
  {
    // Create mutable cloud
    sensor_msgs::PointCloud2::Ptr Cloud = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());

    // Wait for incoming message, update mutable cloud with info from immutable cloud
    *Cloud = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensor_topic_name, ten_seconds);

    if (!Cloud)
    {
      ROS_ERROR("No point cloud received in 3 seconds.");
      return false;
    }
    std::stringstream ss;
    ss << Cloud->header.stamp;
    ROS_INFO("Cloud timestamp: %s", ss.str().c_str());

    // Check whether incoming pointcloud is marked as too old
    if ((ros::Time::now() - ros::Duration(TF_BUFFER_DURATION_SECS - 1)) > Cloud->header.stamp)
    {
      if (update_cloud_timestamp_if_too_old)
      {
        ROS_WARN(
            "Cloud timestamp is too far in the past to compute a tf transformation; overwriting timestamp with Time(0) to get latest transform");
        Cloud->header.stamp = ros::Time(0);
      }
      else
      {
        ROS_ERROR("Cloud timestamp is too far in the past to compute a tf transformation");
        return false;
      }
    }

    // Wait for transform to become available as necessary
    if (!tf_listener->waitForTransform(Cloud->header.frame_id, target_frame_id, Cloud->header.stamp,
                                       ros::Duration(3.0)))
    {
      ROS_ERROR(
          "Transform from '%s' to '%s' not available within timeout duration", Cloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }

    // Do the transform
    if (!pcl_ros::transformPointCloud(target_frame_id, *Cloud, transformedCloud, *tf_listener))
    {
      ROS_ERROR(
          "Unable to transform point cloud from frame '%s' to '%s'", Cloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }
  }

  //call extract objects server
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<rail_pcl_object_segmentation::extract_objects>(
      extract_objects_service_name);

  rail_pcl_object_segmentation::extract_objects extract_srv;

  extract_srv.request.cloud = transformedCloud;
  extract_srv.request.constraints = req.constraints;
  extract_srv.request.plane_slope_tolerance = req.plane_slope_tolerance;

  ROS_INFO("Segmenting image...");
  if (client.call(extract_srv))
  { //successful
    if (should_update_enviroment)
    {
      //Call environment server
      ROS_INFO("Updating environment...");
      ros::ServiceClient env_client = n.serviceClient<rail_object_discovery::UpdateEnvironment>(
          update_environment_service_name);

      rail_object_discovery::UpdateEnvironment env_srv;
      env_srv.request.static_environment = transformedCloud;
      env_srv.request.objects = extract_srv.response.clouds;

      // Copy point clouds for naming
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
        env_srv.request.surfaces.push_back((*it).planeCloud);
      }

      if (env_client.call(env_srv))
      {
        ROS_INFO("Service call complete.");
        // Return named clouds
        res.objects = env_srv.response.objects;
        res.surfaces = env_srv.response.surfaces;

        return true;
      }
      else
      {
        ROS_ERROR("Failed to call environment service");
        return false;
      }
    }
    else
    {
      // Return clouds w/o names
      for (std::vector<sensor_msgs::PointCloud2>::iterator it = extract_srv.response.clouds.begin();
          it != extract_srv.response.clouds.end(); ++it)
      {
        rail_object_discovery::NamedPointCloud2 namedCloud;
        namedCloud.cloud = *it; // Copy point cloud
        res.objects.push_back(namedCloud);
      }

      // Return surfaces w/o names
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
        rail_object_discovery::NamedPointCloud2 namedCloud;
        namedCloud.cloud = (*it).planeCloud; // Copy point cloud
        res.surfaces.push_back(namedCloud);
      }

      ROS_DEBUG("Configured not to update environment; update service call skipped.");
      return true;
    }
  }
  else
  {
    ROS_ERROR("Failed to call extract objects service");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "discover_objects_server");
  ros::NodeHandle priv("~");
  ros::NodeHandle n;

  // Required parameters
  if (!priv.getParam("sensor_topic", sensor_topic_name))
    ROS_ERROR("Parameter '~sensor_topic' must be set.");
  if (!priv.getParam("target_frame_id", target_frame_id))
    ROS_ERROR("Parameter '~target_frame_id' must be set.");

  // Optional parameters
  std::string serviceName;

  priv.param<std::string>("service_name", serviceName, "discover_objects");
  priv.param<std::string>("extract_objects_service_name", extract_objects_service_name, "extract_objects");
  ROS_INFO_STREAM("Expecting extract objects service at name '" << extract_objects_service_name << "'");
  priv.param<bool>("update_environment", should_update_enviroment, true);
  ROS_INFO_STREAM("Will call update environment service: " << should_update_enviroment);
  priv.param<std::string>("update_environment_service_name", update_environment_service_name, "update_environment");
  if (should_update_enviroment)
    ROS_INFO_STREAM("Expecting update environment service at name '" << update_environment_service_name << "'");
  priv.param<bool>("update_old_cloud_timestamps", update_cloud_timestamp_if_too_old, false);
  ROS_INFO_STREAM("Will update old cloud timestamps: " << update_cloud_timestamp_if_too_old);

  ros::ServiceServer service = n.advertiseService(serviceName, discover_objects_callback);
  tf_listener = new tf::TransformListener(ros::Duration(TF_BUFFER_DURATION_SECS));
  ROS_INFO_STREAM("Ready to discover objects at name '" << serviceName << "'");
  ros::spin();

  return 0;
}


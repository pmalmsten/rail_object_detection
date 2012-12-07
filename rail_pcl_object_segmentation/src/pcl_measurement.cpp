#include <cmath>
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"

namespace rail
{

/**
 * \brief Computes the average hue of a point cloud.
 *
 * The hue for a cloud falls within the circular range [0°, 360°).
 *
 * \param [in] cloud The cloud to measure.
 * \return The average hue of all of the points in the cloud; in the circular range 0 - 2pi.
 */
double ComputePointCloudAverageHue(pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr cloud) {
  double total_hue = 0;
  unsigned int total_points = cloud->size();

  for(pcl::PointCloud<pcl::PointXYZHSV>::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {
    double hue = (*it).h;

    // Due to the definition of hue, some points may not have a hue (i.e. they are some shade of black).
    // These are ignored during color analysis.
    if(!isnan(hue))
      total_hue += hue;
    else
      total_points--;
  }

  return total_hue / total_points;
}

/**
 * \brief Computes the average hue of a point cloud.
 *
 * The given point cloud of XYZRGB points is converted to a cloud of XYZHSV before
 * analysis.
 *
 * The hue for a cloud falls within the circular range [0°, 360°).
 *
 * \param [in] cloud The cloud to measure.
 * \return The average hue of all of the points in the cloud; in the circular range 0 - 2pi
 */
double ComputePointCloudAverageHue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud = pcl::PointCloud<pcl::PointXYZHSV>::Ptr(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *hsvCloud);

  return ComputePointCloudAverageHue((pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr) hsvCloud);
}

/**
 * \brief Determines whether the given point cloud satisfies the range in this is_correct_color structure.
 */
bool is_correct_color::operator() (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const
{
  double avg_hue = ComputePointCloudAverageHue(cloud);

  if(!invert_range) {
    return avg_hue >= min_hue && avg_hue <= max_hue;
  } else {
    return avg_hue < min_hue || avg_hue > max_hue;
  }
}

/**
 * \brief Determines whether the given plane satisfies the tolerance in this is_level structure.
 */
bool is_level::operator()(const rail::DiscoveredPlanePtr plane) const
{
  if (fabs(plane->a) > max_slope)
    return false;

  if (fabs(plane->b) > max_slope)
    return false;

  return true;
}

/**
 * \brief Removes point clouds which are not level.
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

/**
 * \brief Removes point clouds with a hue that does not fall within the correct range.
 *
 * \param [inout] pointClouds The vector of point clouds to filer in-place.
 * \param [in] min_hue The minimum hue value that a cloud may present.
 * \param [in] max_hue The maximum hue value that a cloud may present.
 * \param invert_range When false, only clouds with an average hue in the specified range are returned. If false, only
 *  only clouds with an average hue outside of the specified range are returned.
 */
void FilterWrongColor(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& pointClouds, double min_hue, double max_hue, bool invert_range) {
  struct is_correct_color is_right_color;
  is_right_color.min_hue = min_hue;
  is_right_color.max_hue = max_hue;
  is_right_color.invert_range = invert_range;

  pointClouds.erase(std::remove_if(pointClouds.begin(), pointClouds.end(), std::not1(is_right_color)),
                    pointClouds.end());
}
}

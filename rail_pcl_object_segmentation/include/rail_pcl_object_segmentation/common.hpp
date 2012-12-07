#ifndef _RAIL_PCL_OBJECT_SEGMENTATION_COMMON_H
#define _RAIL_PCL_OBJECT_SEGMENTATION_COMMON_H

/*!
 * \file common.hpp
 * \brief Header file for common definitions.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@wpi.edu
 * \date Nov 28, 2012
 */

#include "rail_pcl_object_segmentation/DiscoveredPlane.h"

namespace rail
{
typedef boost::shared_ptr<rail_pcl_object_segmentation::DiscoveredPlane> DiscoveredPlanePtr;
}

#endif

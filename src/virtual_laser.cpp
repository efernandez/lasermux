/*********************************************************************
 * Copyright (c) 2014 Paul Mathieu
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 *   1. The origin of this software must not be misrepresented; you must not
 *   claim that you wrote the original software. If you use this software
 *   in a product, an acknowledgment in the product documentation would be
 *   appreciated but is not required.
 *
 *   2. Altered source versions must be plainly marked as such, and must not be
 *   misrepresented as being the original software.
 *
 *   3. This notice may not be removed or altered from any source
 *   distribution.
 *********************************************************************/


#include "lasermux/virtual_laser.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

namespace lasermux
{

VirtualLaser::VirtualLaser(const ros::NodeHandle& nh)
  : nh_(nh)
  , tfl_(nh)
{ }

VirtualLaser::~VirtualLaser()
{ }

void VirtualLaser::setParams(
    const std::string& frame_id,
    double range_min,
    double range_max,
    double angle_min,
    double angle_max,
    double angle_increment
    )
{
  frame_id_ = frame_id;
  range_min_ = range_min;
  range_max_ = range_max;
  angle_min_ = angle_min;
  angle_max_ = angle_max;
  angle_increment_ = angle_increment;
}

void VirtualLaser::takePointCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  // 1. filter point cloud
  // 1.1 project point cloud on frame_id
  // 1.2 project points on conic segments
  // 1.3 update laserScan
}

void VirtualLaser::takeLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
{
  ros::Time scan_time = scan->header.stamp +
    ros::Duration(scan->ranges.size()*scan->time_increment);
  if (!tfl_.canTransform(scan->header.frame_id, frame_id_, scan_time))
  {
    ROS_ERROR_STREAM("Can not transform laser scan from frame "
        << scan->header.frame_id << " at time " << scan_time);
    return;
  }

  sensor_msgs::PointCloud2* cloud = new sensor_msgs::PointCloud2();
  projector_.transformLaserScanToPointCloud(frame_id_, *scan, *cloud, tfl_);

  takePointCloud(sensor_msgs::PointCloud2ConstPtr(cloud));
}

sensor_msgs::LaserScanConstPtr VirtualLaser::generateScan()
{
  return sensor_msgs::LaserScanConstPtr(latest_scan_);
}

}

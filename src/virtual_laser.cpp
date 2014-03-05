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
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <math.h>

#define foreach BOOST_FOREACH

namespace lasermux
{

VirtualLaser::VirtualLaser(const ros::NodeHandle& nh)
  : nh_(nh)
  , tfl_(nh)
{
  latest_scan_.reset(new sensor_msgs::LaserScan);
}

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

  latest_scan_->ranges.resize((int)(angle_max_ - angle_min_)/angle_increment_ + 1);
  latest_scan_->range_min = range_min_;
  latest_scan_->range_max = range_max_;
  latest_scan_->angle_min = angle_min_;
  latest_scan_->angle_max = angle_max_;
  latest_scan_->angle_increment = angle_increment_;
  latest_scan_->header.frame_id = frame_id_;

  scan_filter_.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(tfl_, frame_id_, 10, nh_));
  pc_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(tfl_, frame_id_, 10, nh_));

  scan_filter_->registerCallback(&VirtualLaser::scanTransformReady, this);
  pc_filter_->registerCallback(&VirtualLaser::pcTransformReady, this);
}

void VirtualLaser::scanTransformReady(const sensor_msgs::LaserScanConstPtr& scan)
{
  sensor_msgs::PointCloud2* cloud = new sensor_msgs::PointCloud2;
  projector_.transformLaserScanToPointCloud(frame_id_, *scan, *cloud, tfl_);
  takePointCloud(sensor_msgs::PointCloud2ConstPtr(cloud));
}

void VirtualLaser::pcTransformReady(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  sensor_msgs::PointCloud2* cloud = new sensor_msgs::PointCloud2;
  if(!pcl_ros::transformPointCloud(frame_id_, *pc, *cloud, tfl_))
  {
    ROS_ERROR("Could not transform point cloud");
    return;
  }

  updateScanWithPC(sensor_msgs::PointCloud2ConstPtr(cloud));
}

void VirtualLaser::updateScanWithPC(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*pc, cloud);

  if (latest_scan_->ranges.size() != reading_age_.size())
  {
    reading_age_.clear();
    reading_age_.resize(latest_scan_->ranges.size(), 0);
  }

  foreach (const pcl::PointXYZ& p, cloud)
  {
    double ang = atan2(p.y, p.x);
    double min = angle_min_ - angle_increment_/2;
    double max = angle_max_ + angle_increment_/2;
    double dist = sqrt(p.x*p.x + p.y*p.y);
    double max_height = angle_increment_/2 * dist;
    double reading = 0.0;

    // discard points outside of the aperture
    if (ang < min || ang > max)
    {
      continue;
    }

    reading = dist;

    // update scan
    int index = (int)((ang - angle_min_) / angle_increment_ + 0.5);
    latest_scan_->ranges[index] = reading;
    reading_age_[index] = 0;

  }
  latest_scan_->header.stamp = ros::Time::now();

  // garbage-collecting
  for (int i = 0; i < reading_age_.size(); ++i)
  {
    if (reading_age_[i] > 4)  // TODO: set this as a param
    {
      latest_scan_->ranges[i] = 0.0;
    }
    ++reading_age_[i];
  }
}

void VirtualLaser::takePointCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  std::string our_frame = tf::resolve("/", frame_id_);
  std::string pc_frame = tf::resolve("/", pc->header.frame_id);
  sensor_msgs::PointCloud2ConstPtr cloud = pc;
  if (our_frame == pc_frame)
  {
    // don't transform to the same frame
    updateScanWithPC(cloud);
  }
  else
  {
    pc_filter_->add(pc);
  }
}

void VirtualLaser::takeLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
{
  ros::Time scan_time = scan->header.stamp +
    ros::Duration(scan->ranges.size()*scan->time_increment);
  std::string err;
  if (tfl_.canTransform(frame_id_, scan->header.frame_id, scan_time, &err))
  {
    // don't wait if the transform is already available
    scanTransformReady(scan);
  }
  else
  {
    scan_filter_->add(scan);
  }
}

sensor_msgs::LaserScanConstPtr VirtualLaser::generateScan()
{
  return sensor_msgs::LaserScanConstPtr(latest_scan_);
}

}

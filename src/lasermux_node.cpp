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
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <boost/bind.hpp>

class LaserMuxNode
{
public:
  LaserMuxNode(const ros::NodeHandle& nh = ros::NodeHandle("~"))
  : nh_(nh)
  {
    vl_.setParams("base_footprint", 1.0, 10.0, -2.0, 2.0, 0.02);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_out", 10);
    tmr_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&LaserMuxNode::publish, this));
    sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scans_in", 10,
        boost::bind(&LaserMuxNode::scansCB, this, _1));
  }

private:
  void scansCB(const sensor_msgs::LaserScanConstPtr& scan)
  {
    vl_.takeLaserScan(scan);
  }

  void publish()
  {
    pub_.publish(vl_.generateScan());
  }

  ros::NodeHandle nh_;
  lasermux::VirtualLaser vl_;
  ros::Publisher pub_;
  ros::Timer tmr_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lasermux");

  // TODO: get parameters from rosparam
  // TODO: get parameters from dynamic reconfigure
  LaserMuxNode node;

  ros::spin();
}

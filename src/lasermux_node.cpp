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
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

class LaserMuxNode
{
public:
  LaserMuxNode(const ros::NodeHandle& nh = ros::NodeHandle("~"))
  : nh_(nh)
  {
    std::string frame_id = "base_footprint";
    double range_min = 0.0, range_max = 100.0;
    double z_min = -100.0, z_max = 100.0;
    double angle_min = -M_PI, angle_max = M_PI, angle_increment = 0.01;
    double pitch_abs_max = M_PI, roll_abs_max = M_PI;
    double frequency = 10.0;
    int max_reading_age = 1;
    double tolerance = 0.0;

    nh_.param("frame_id", frame_id, frame_id);
    nh_.param("z_min", z_min, z_min);
    nh_.param("z_max", z_max, z_max);
    nh_.param("range_min", range_min, range_min);
    nh_.param("range_max", range_max, range_max);
    nh_.param("angle_min", angle_min, angle_min);
    nh_.param("angle_max", angle_max, angle_max);
    nh_.param("angle_increment", angle_increment, angle_increment);
    nh_.param("pitch_abs_max", pitch_abs_max, pitch_abs_max);
    nh_.param("roll_abs_max", roll_abs_max, roll_abs_max);
    nh_.param("frequency", frequency, frequency);
    nh_.param("max_reading_age", max_reading_age, max_reading_age);
    nh_.param("tolerance", tolerance, tolerance);

    vl_.setParams(frame_id,
        range_min, range_max, z_min, z_max,
        angle_min, angle_max, angle_increment,
        pitch_abs_max, roll_abs_max,
        max_reading_age, tolerance);

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_out", 10);
    tmr_ = nh_.createTimer(ros::Duration(1./frequency),
        boost::bind(&LaserMuxNode::publish, this));

    std::vector<std::string> scans_in;
    if (nh_.getParam("scans_in", scans_in))
    {
      foreach (const std::string& scan_topic, scans_in)
      {
        sub_.push_back(root_nh_.subscribe<sensor_msgs::LaserScan>(scan_topic, 10,
                       boost::bind(&LaserMuxNode::scansCB, this, _1)));
      }
    }
    else
    {
      ROS_ERROR("No scans_in list provided, so no scans will be received!");
    }
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

  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  lasermux::VirtualLaser vl_;
  ros::Publisher pub_;
  ros::Timer tmr_;
  std::vector<ros::Subscriber> sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lasermux");

  // TODO: get parameters from rosparam
  // TODO: get parameters from dynamic reconfigure
  LaserMuxNode node;

  ros::spin();
}

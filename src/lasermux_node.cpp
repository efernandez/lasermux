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
    vl_.setParams("base_footprint", 0.01, 10.0, -2.0, 2.0, 0.05);
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

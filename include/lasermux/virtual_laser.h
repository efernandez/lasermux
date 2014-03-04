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


#include <ros/node_handle.h>
#include <ros/message_forward.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(PointCloud2);
ROS_DECLARE_MESSAGE(LaserScan);
}

namespace lasermux
{

class VirtualLaser
{
public:
    VirtualLaser(const ros::NodeHandle& nh = ros::NodeHandle());
    virtual ~VirtualLaser();

    /**
     * \brief Set virtual laser parameters. See sensor_msgs::LaserScan
     */
    void setParams(
            const std::string& frame_id,
            double range_min,
            double range_max,
            double angle_min,
            double angle_max,
            double angle_increment
            );

    /**
     * @brief Update virtual laser with a point cloud
     * @param pc PointCloud2 message
     */
    void takePointCloud(const sensor_msgs::PointCloud2ConstPtr& pc);

    /**
     * @brief Update virtual laser with a laser scan
     * @param scan LaserScan message
     */
    void takeLaserScan(const sensor_msgs::LaserScanConstPtr& scan);

    /**
     * @brief Generate a LaserScan message as if read from the virtual sensor
     * @return A LaserScan message, ready to publish
     */
    sensor_msgs::LaserScanConstPtr generateScan();

private:
    // latest ready-to-publish scan
    sensor_msgs::LaserScanPtr latest_scan_;

    // virtual scanner params
    std::string frame_id_;
    double      range_min_;
    double      range_max_;
    double      angle_min_;
    double      angle_max_;
    double      angle_increment_;

    ros::NodeHandle nh_;
    tf::TransformListener tfl_;
    laser_geometry::LaserProjection projector_;
};

}

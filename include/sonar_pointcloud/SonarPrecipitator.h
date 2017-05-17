/**
    SonarPrecipitator Class
    SonarPrecipitator.h
    Purpose: Class that converts `range msg` and `transforms` into PointCloud2

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SONAR_PRECIPITATOR_H
#define SONAR_PRECIPITATOR_H

#include "sonar_pointcloud.h"

#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

namespace sonar_pointcloud {

class SonarPrecipitator {
public:
    SonarPrecipitator(const std::string& pointcloudTopic, const std::string& pointcloudFrame);
    ~SonarPrecipitator() {}

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher pointcloudPublisher;

    std::string pointcloudFrame;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

}

#endif

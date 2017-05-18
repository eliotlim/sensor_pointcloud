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
#include "sonar_pointcloud/Sonar.h"

#include <boost/thread.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace sonar_pointcloud {

class SonarPrecipitator {
public:
    SonarPrecipitator(const std::string& pointcloudTopic, const std::string& pointcloudFrame);
    ~SonarPrecipitator() {}
    void addSonar(const std::string& sonarTopic, const std::string& sonarFrame);

private:
    void publishCallable();

    ros::NodeHandle nodeHandle;
    ros::Publisher pointcloudPublisher;
    boost::thread publishThread;
    double publishRate;

    std::string frame;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::vector<sonar_pointcloud::Sonar> sonars;

};

}

#endif

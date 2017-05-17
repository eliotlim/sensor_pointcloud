/**
    SonarPrecipitator Class
    SonarPrecipitator.h
    Purpose: Class that converts `range msg` and `transforms` into PointCloud2

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/SonarPrecipitator.h>

using namespace sonar_pointcloud;

/**
    Constructor for SonarPrecipitator
    Instantiates pointcloudFrame and tfListener using a tfBuffer

    @param pointcloudTopic Output topic for PointCloud2
    @param pointcloudFrame TF2 Frame for point cloud origin
*/

SonarPrecipitator::SonarPrecipitator(const std::string& pointcloudTopic, const std::string& pointcloudFrame) :
                                     pointcloudFrame(pointcloudFrame), tfListener(tfBuffer) {
    nodeHandle = ros::NodeHandle();
    pointcloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2> (pointcloudTopic, 3);
}

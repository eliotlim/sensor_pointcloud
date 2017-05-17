/**
    SonarPrecipitator Class
    SonarPrecipitator.h
    Purpose: Class that converts `range msg` and `transforms` into PointCloud2

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

using namespace sonar_pointcloud;

/**
    Constructor for SonarPrecipitator

    @param pointcloudTopic Output topic for PointCloud2
    @param pointcloudFrame TF2 Frame for point cloud origin
*/

SonarPrecipitator::SonarPrecipitator(std::string pointcloudTopic, std::string pointcloudFrame) {
    nodeHandle = ros::NodeHandle();

}

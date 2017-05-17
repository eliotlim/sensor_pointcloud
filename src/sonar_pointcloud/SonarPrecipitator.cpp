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
    // ROS Setup
    nodeHandle = ros::NodeHandle();
    pointcloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2> (pointcloudTopic, 3);

    // Start publisher thread
    // Refer to http://stackoverflow.com/questions/4581476/using-boost-thread-and-a-non-static-class-function
    publishThread = boost::thread(boost::bind(&SonarPrecipitator::publishCallable, this));
}

/**
    Add a Sonar Input by Topic
    Instantiates a Sonar Object

    @param sonarTopic Input topic for `range_msg`
    @param pointcloudFrame TF2 Frame for Sonar Orientation
*/

void SonarPrecipitator::addSonar(const std::string& sonarTopic, const std::string& sonarFrame) {
    sonars.push_back(Sonar(sonarTopic, sonarFrame));
}

/**
    Publish thread callable
    Loops at specified publishRate, publishing PointCloud2 messages
    Transform Sonar range reading to point cloud about pointcloudFrame
*/

void SonarPrecipitator::publishCallable() {
    ros::Rate loopRate(publishRate);
    while (ros::ok()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pointCloud->header.frame_id = pointcloudFrame;
        pointCloud->height = 1;

        // TODO: Convert all Sonar readings to Points


        // Publish PointCloud2
        pointcloudPublisher.publish(pointCloud);
        loopRate.sleep();
    }
}
